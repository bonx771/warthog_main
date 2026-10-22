#!/usr/bin/env python3
"""Acceleration limiter used only by outdoor_waypoint_obstacle.launch.

Normal velocity changes, including short zero commands from move_base, are
ramped.  A stale input or a false localization-ready signal still stops
immediately.  This prevents one-cycle planner zeros from repeatedly resetting
the heavy UGV to zero speed.
"""

import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def step_towards(current, target, accel_limit, decel_limit, dt):
    """Move current toward a non-zero target with bounded acceleration."""
    if dt <= 0.0 or current == target:
        return current

    same_direction = current == 0.0 or current * target > 0.0
    speeding_up = same_direction and abs(target) > abs(current)
    limit = accel_limit if speeding_up else decel_limit
    max_step = max(0.0, limit) * dt
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + max_step * (1.0 if delta > 0.0 else -1.0)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class ObstacleCmdVelSmoother:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/cmd_vel_raw_obstacle"
        )
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.frequency = float(rospy.get_param("~frequency", 30.0))
        self.linear_accel = float(rospy.get_param("~linear_accel", 0.35))
        self.linear_decel = float(rospy.get_param("~linear_decel", 0.65))
        self.angular_accel = float(rospy.get_param("~angular_accel", 0.55))
        self.angular_decel = float(rospy.get_param("~angular_decel", 0.90))
        self.max_linear = float(rospy.get_param("~max_linear", 0.70))
        self.max_angular = float(rospy.get_param("~max_angular", 0.35))
        self.angular_startup_floor = float(
            rospy.get_param("~angular_startup_floor", 0.0)
        )
        self.enforce_in_place_angular_floor = bool(
            rospy.get_param("~enforce_in_place_angular_floor", False)
        )
        self.in_place_linear_threshold = float(
            rospy.get_param("~in_place_linear_threshold", 0.02)
        )
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.50))
        self.deadband = float(rospy.get_param("~deadband", 0.005))
        self.smooth_zero_commands = bool(
            rospy.get_param("~smooth_zero_commands", True)
        )
        self.navigation_ready_topic = rospy.get_param(
            "~navigation_ready_topic",
            "/outdoor_waypoint_nav/navigation_ready_obstacle",
        )
        self.require_navigation_ready = bool(
            rospy.get_param("~require_navigation_ready", True)
        )
        self.heading_control_enabled = bool(
            rospy.get_param("~heading_control_enabled", True)
        )
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/outdoor_waypoint_nav/odometry/filtered"
        )
        self.global_plan_topic = rospy.get_param(
            "~global_plan_topic", "/move_base/TrajectoryPlannerROS/global_plan"
        )
        self.heading_lookahead = float(
            rospy.get_param("~heading_lookahead", 1.0)
        )
        self.heading_data_timeout = float(
            rospy.get_param("~heading_data_timeout", 0.75)
        )
        self.drive_enter_slow_angle = math.radians(
            float(rospy.get_param("~drive_enter_slow_angle_deg", 35.0))
        )
        self.slow_exit_drive_angle = math.radians(
            float(rospy.get_param("~slow_exit_drive_angle_deg", 25.0))
        )
        self.align_enter_angle = math.radians(
            float(rospy.get_param("~align_enter_angle_deg", 45.0))
        )
        self.align_exit_angle = math.radians(
            float(rospy.get_param("~align_exit_angle_deg", 35.0))
        )
        self.slow_curve_max_linear = float(
            rospy.get_param("~slow_curve_max_linear", 0.30)
        )
        self.transition_linear_threshold = float(
            rospy.get_param("~transition_linear_threshold", 0.05)
        )
        self.transition_angular_threshold = float(
            rospy.get_param("~transition_angular_threshold", 0.05)
        )
        self.align_min_angular = float(
            rospy.get_param("~align_min_angular", 0.25)
        )
        self.scan_topic = rospy.get_param("~scan_topic", "/front/scan")
        self.align_min_scan_clearance = float(
            rospy.get_param("~align_min_scan_clearance", 1.15)
        )
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.50))

        if self.frequency <= 0.0:
            raise ValueError("frequency must be positive")
        for name, value in (
            ("linear_accel", self.linear_accel),
            ("linear_decel", self.linear_decel),
            ("angular_accel", self.angular_accel),
            ("angular_decel", self.angular_decel),
            ("max_linear", self.max_linear),
            ("max_angular", self.max_angular),
            ("command_timeout", self.command_timeout),
        ):
            if value <= 0.0:
                raise ValueError("{} must be positive".format(name))
        self.angular_startup_floor = min(
            self.max_angular, max(0.0, self.angular_startup_floor)
        )
        self.align_min_angular = min(
            self.max_angular, max(0.0, self.align_min_angular)
        )
        self.in_place_linear_threshold = max(
            0.0, self.in_place_linear_threshold
        )
        self.heading_lookahead = max(0.10, self.heading_lookahead)
        self.heading_data_timeout = max(0.10, self.heading_data_timeout)
        self.slow_curve_max_linear = min(
            self.max_linear, max(0.0, self.slow_curve_max_linear)
        )
        if not (
            0.0 <= self.slow_exit_drive_angle <= self.drive_enter_slow_angle
            < self.align_enter_angle
        ):
            raise ValueError(
                "heading thresholds must satisfy 0 <= slow_exit <= "
                "drive_enter_slow < align_enter"
            )
        if not 0.0 <= self.align_exit_angle < self.align_enter_angle:
            raise ValueError("align_exit must be smaller than align_enter")

        self.lock = threading.Lock()
        self.motion_state = "DRIVE"
        self.explicit_align = False
        self.raw_linear = 0.0
        self.raw_angular = 0.0
        self.latest_odom = None
        self.latest_plan = None
        self.latest_scan_clearance = None
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.last_command_time = None
        self.last_update_time = time.monotonic()
        self.navigation_ready = not self.require_navigation_ready

        self.publisher = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.ready_subscriber = rospy.Subscriber(
            self.navigation_ready_topic,
            Bool,
            self.navigation_ready_callback,
            queue_size=1,
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, Twist, self.command_callback, queue_size=1
        )
        self.odom_subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        self.plan_subscriber = rospy.Subscriber(
            self.global_plan_topic, Path, self.plan_callback, queue_size=1
        )
        self.scan_subscriber = rospy.Subscriber(
            self.scan_topic, LaserScan, self.scan_callback, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.frequency), self.update
        )
        rospy.on_shutdown(self.stop_now)

        rospy.loginfo(
            "[cmd_vel_smoother_obstacle] %s -> %s at %.1f Hz | "
            "linear accel/decel=%.2f/%.2f m/s^2 | "
            "angular accel/decel=%.2f/%.2f rad/s^2 | "
            "turn startup floor=%.2f rad/s | continuous turn floor=%s | "
            "planner-zero=%s | heading states=%s (drive<%.1fdeg, "
            "slow<=%.2fm/s, align>%.1fdeg, exit<%.1fdeg)",
            self.input_topic,
            self.output_topic,
            self.frequency,
            self.linear_accel,
            self.linear_decel,
            self.angular_accel,
            self.angular_decel,
            self.angular_startup_floor,
            "enabled" if self.enforce_in_place_angular_floor else "disabled",
            "ramped" if self.smooth_zero_commands else "immediate",
            "enabled" if self.heading_control_enabled else "disabled",
            math.degrees(self.drive_enter_slow_angle),
            self.slow_curve_max_linear,
            math.degrees(self.align_enter_angle),
            math.degrees(self.align_exit_angle),
        )
        if self.require_navigation_ready:
            rospy.logwarn(
                "[cmd_vel_smoother_obstacle] Velocity LOCKED until %s is true.",
                self.navigation_ready_topic,
            )

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def normalized_frame(frame_id):
        return frame_id.lstrip("/")

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw_from_quaternion(msg.pose.pose.orientation),
                self.normalized_frame(msg.header.frame_id),
                time.monotonic(),
            )

    def plan_callback(self, msg):
        if not msg.poses:
            return
        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        with self.lock:
            self.latest_plan = (
                points,
                self.normalized_frame(msg.header.frame_id),
                time.monotonic(),
            )

    def scan_callback(self, msg):
        usable_ranges = [
            float(value)
            for value in msg.ranges
            if math.isfinite(value)
            and value >= msg.range_min
            and value <= msg.range_max
        ]
        clearance = min(usable_ranges) if usable_ranges else float("inf")
        with self.lock:
            self.latest_scan_clearance = (clearance, time.monotonic())

    def align_clearance_available_locked(self, now):
        if self.align_min_scan_clearance <= 0.0:
            return True
        if self.latest_scan_clearance is None:
            return False
        clearance, received_at = self.latest_scan_clearance
        return (
            now - received_at <= self.scan_timeout
            and clearance >= self.align_min_scan_clearance
        )

    def heading_error_locked(self, now):
        if not self.heading_control_enabled:
            return None
        if self.latest_odom is None or self.latest_plan is None:
            return None

        robot_x, robot_y, robot_yaw, odom_frame, odom_time = self.latest_odom
        points, plan_frame, plan_time = self.latest_plan
        if (
            now - odom_time > self.heading_data_timeout
            or now - plan_time > self.heading_data_timeout
            or odom_frame != plan_frame
        ):
            return None

        nearest_index = min(
            range(len(points)),
            key=lambda index: (
                (points[index][0] - robot_x) ** 2
                + (points[index][1] - robot_y) ** 2
            ),
        )
        target_index = nearest_index
        accumulated = 0.0
        for index in range(nearest_index + 1, len(points)):
            accumulated += math.hypot(
                points[index][0] - points[index - 1][0],
                points[index][1] - points[index - 1][1],
            )
            target_index = index
            if accumulated >= self.heading_lookahead:
                break

        target_x, target_y = points[target_index]
        dx = target_x - robot_x
        dy = target_y - robot_y
        if math.hypot(dx, dy) < 0.05:
            return None
        return normalize_angle(math.atan2(dy, dx) - robot_yaw)

    def set_motion_state_locked(self, new_state, heading_error=None):
        if new_state == self.motion_state:
            return
        old_state = self.motion_state
        self.motion_state = new_state
        if heading_error is None:
            rospy.loginfo(
                "[cmd_vel_smoother_obstacle] motion state %s -> %s",
                old_state,
                new_state,
            )
        else:
            rospy.loginfo(
                "[cmd_vel_smoother_obstacle] motion state %s -> %s "
                "(heading error %.1f deg)",
                old_state,
                new_state,
                math.degrees(heading_error),
            )

    def apply_motion_state_targets_locked(self, heading_error):
        if self.motion_state == "DRIVE":
            self.target_linear = self.raw_linear
            self.target_angular = self.raw_angular
        elif self.motion_state == "SLOW_CURVE":
            self.target_linear = self.clamp(
                self.raw_linear, self.slow_curve_max_linear
            )
            self.target_angular = self.raw_angular
        elif self.motion_state == "ALIGN":
            self.target_linear = 0.0
            angular = self.raw_angular
            if self.explicit_align and abs(angular) > self.deadband:
                magnitude = min(
                    self.max_angular,
                    max(abs(angular), self.align_min_angular),
                )
                angular = magnitude if angular > 0.0 else -magnitude
            elif heading_error is not None:
                magnitude = min(
                    self.max_angular,
                    max(abs(angular), self.align_min_angular),
                )
                angular = magnitude if heading_error > 0.0 else -magnitude
            self.target_angular = angular
        else:  # ALIGN_BRAKE or ALIGN_RELEASE
            self.target_linear = 0.0
            self.target_angular = 0.0

    def select_motion_state_locked(self, now, explicit_in_place_turn):
        heading_error = self.heading_error_locked(now)

        if self.motion_state == "ALIGN_RELEASE":
            self.apply_motion_state_targets_locked(heading_error)
            return heading_error

        if explicit_in_place_turn:
            self.explicit_align = True
            new_state = (
                "ALIGN_BRAKE"
                if abs(self.output_linear) > self.transition_linear_threshold
                else "ALIGN"
            )
            self.set_motion_state_locked(new_state, heading_error)
            self.apply_motion_state_targets_locked(heading_error)
            return heading_error

        self.explicit_align = False
        if heading_error is None:
            if self.motion_state in ("ALIGN", "ALIGN_BRAKE"):
                self.set_motion_state_locked("ALIGN_RELEASE")
            else:
                self.set_motion_state_locked("DRIVE")
            self.apply_motion_state_targets_locked(heading_error)
            return heading_error

        abs_error = abs(heading_error)
        if self.motion_state in ("ALIGN", "ALIGN_BRAKE"):
            if not self.align_clearance_available_locked(now):
                self.set_motion_state_locked("ALIGN_RELEASE", heading_error)
                rospy.logwarn_throttle(
                    1.0,
                    "[cmd_vel_smoother_obstacle] ALIGN released because "
                    "the in-place rotation clearance is unavailable.",
                )
            elif abs_error <= self.align_exit_angle:
                self.set_motion_state_locked("ALIGN_RELEASE", heading_error)
        elif abs_error >= self.align_enter_angle:
            if self.align_clearance_available_locked(now):
                new_state = (
                    "ALIGN_BRAKE"
                    if abs(self.output_linear)
                    > self.transition_linear_threshold
                    else "ALIGN"
                )
                self.set_motion_state_locked(new_state, heading_error)
            else:
                self.set_motion_state_locked("SLOW_CURVE", heading_error)
                rospy.logwarn_throttle(
                    1.0,
                    "[cmd_vel_smoother_obstacle] ALIGN skipped because "
                    "the in-place rotation clearance is unavailable; "
                    "using SLOW_CURVE.",
                )
        elif self.motion_state == "SLOW_CURVE":
            if abs_error <= self.slow_exit_drive_angle:
                self.set_motion_state_locked("DRIVE", heading_error)
        elif abs_error >= self.drive_enter_slow_angle:
            self.set_motion_state_locked("SLOW_CURVE", heading_error)
        else:
            self.set_motion_state_locked("DRIVE", heading_error)

        self.apply_motion_state_targets_locked(heading_error)
        return heading_error

    def navigation_ready_callback(self, msg):
        ready = bool(msg.data) or not self.require_navigation_ready
        with self.lock:
            changed = ready != self.navigation_ready
            self.navigation_ready = ready
            if not ready:
                self.motion_state = "DRIVE"
                self.explicit_align = False
                self.raw_linear = 0.0
                self.raw_angular = 0.0
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.output_linear = 0.0
                self.output_angular = 0.0
                self.last_command_time = None

        if not ready:
            # The callback path provides an immediate stop; the timer continues
            # sending zero while localization remains unavailable.
            self.publisher.publish(Twist())
            if changed:
                rospy.logerr(
                    "[cmd_vel_smoother_obstacle] Velocity LOCKED by localization guard."
                )
        elif changed:
            rospy.loginfo(
                "[cmd_vel_smoother_obstacle] Localization ready; velocity ENABLED."
            )

    def command_callback(self, msg):
        now = time.monotonic()
        linear = self.clamp(float(msg.linear.x), self.max_linear)
        angular = self.clamp(float(msg.angular.z), self.max_angular)
        in_place_turn = (
            abs(linear) <= self.in_place_linear_threshold
            and abs(angular) > self.deadband
        )
        if (
            self.enforce_in_place_angular_floor
            and in_place_turn
            and abs(angular) < self.angular_startup_floor
        ):
            angular = (
                self.angular_startup_floor if angular > 0.0
                else -self.angular_startup_floor
            )
        is_zero = abs(linear) <= self.deadband and abs(angular) <= self.deadband

        with self.lock:
            if not self.navigation_ready:
                self.last_command_time = None
                return
            self.last_command_time = now
            self.raw_linear = linear
            self.raw_angular = angular
            if is_zero:
                self.target_linear = 0.0
                self.target_angular = 0.0
                if self.motion_state in ("ALIGN", "ALIGN_BRAKE"):
                    self.set_motion_state_locked("ALIGN_RELEASE")
                if not self.smooth_zero_commands:
                    self.output_linear = 0.0
                    self.output_angular = 0.0
            else:
                self.select_motion_state_locked(now, in_place_turn)

        if is_zero and not self.smooth_zero_commands:
            self.publisher.publish(Twist())

    def update(self, _event):
        now = time.monotonic()
        with self.lock:
            dt = min(max(now - self.last_update_time, 0.0), 0.20)
            self.last_update_time = now

            stale = (
                not self.navigation_ready
                or self.last_command_time is None
                or now - self.last_command_time >= self.command_timeout
            )
            if stale:
                self.motion_state = "DRIVE"
                self.explicit_align = False
                self.raw_linear = 0.0
                self.raw_angular = 0.0
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.output_linear = 0.0
                self.output_angular = 0.0
            else:
                heading_error = self.heading_error_locked(now)
                if (
                    self.motion_state == "ALIGN_BRAKE"
                    and abs(self.output_linear)
                    <= self.transition_linear_threshold
                ):
                    self.set_motion_state_locked("ALIGN", heading_error)
                    self.apply_motion_state_targets_locked(heading_error)
                elif (
                    self.motion_state == "ALIGN_RELEASE"
                    and abs(self.output_linear)
                    <= self.transition_linear_threshold
                    and abs(self.output_angular)
                    <= self.transition_angular_threshold
                ):
                    self.output_linear = 0.0
                    self.output_angular = 0.0
                    if (
                        heading_error is not None
                        and abs(heading_error) >= self.drive_enter_slow_angle
                    ):
                        self.set_motion_state_locked(
                            "SLOW_CURVE", heading_error
                        )
                    else:
                        self.set_motion_state_locked("DRIVE", heading_error)
                    self.apply_motion_state_targets_locked(heading_error)

                self.output_linear = step_towards(
                    self.output_linear,
                    self.target_linear,
                    self.linear_accel,
                    self.linear_decel,
                    dt,
                )
                starting_in_place_turn = (
                    self.angular_startup_floor > 0.0
                    and abs(self.output_angular) <= self.deadband
                    and abs(self.target_linear) <= self.in_place_linear_threshold
                    and abs(self.target_angular)
                    >= self.angular_startup_floor
                )
                if starting_in_place_turn:
                    self.output_angular = (
                        self.angular_startup_floor
                        if self.target_angular > 0.0
                        else -self.angular_startup_floor
                    )
                else:
                    self.output_angular = step_towards(
                        self.output_angular,
                        self.target_angular,
                        self.angular_accel,
                        self.angular_decel,
                        dt,
                    )

            output = Twist()
            output.linear.x = self.output_linear
            output.angular.z = self.output_angular

        self.publisher.publish(output)

    def stop_now(self):
        stop = Twist()
        for _ in range(3):
            self.publisher.publish(stop)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_smoother_obstacle")
    ObstacleCmdVelSmoother()
    rospy.spin()
