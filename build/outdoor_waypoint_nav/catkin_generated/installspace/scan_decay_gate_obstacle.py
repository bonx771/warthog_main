#!/usr/bin/env python3
"""Gate obstacle scans and expire their costmap marks after a short TTL.

The live scan is forwarded only while localization_guard_obstacle permits
costmap input.  A copy of every forwarded scan is retained for ``decay_time``
seconds and then published on a clearing-only topic with the original stamp.
Keeping that stamp makes costmap_2d use the same historical TF that originally
marked the cells, so old marks are removed even after map->odom has moved.
"""

import collections
import copy
import math
import threading
import time

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ScanDecayGateObstacle:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/front/scan_raw")
        self.output_topic = rospy.get_param("~output_topic", "/front/scan")
        self.clearing_topic = rospy.get_param(
            "~clearing_topic", "/front/scan_expired"
        )
        self.enable_topic = rospy.get_param(
            "~enable_topic",
            "/outdoor_waypoint_nav/lidar_costmap_enabled_obstacle",
        )
        self.decay_time = float(rospy.get_param("~decay_time", 4.5))
        self.clear_extension = float(rospy.get_param("~clear_extension", 0.60))
        self.timer_period = float(rospy.get_param("~timer_period", 0.02))
        self.max_pending_scans = int(rospy.get_param("~max_pending_scans", 300))

        if self.decay_time <= 0.0:
            raise ValueError("decay_time must be positive")
        if self.clear_extension <= 0.0:
            raise ValueError("clear_extension must be positive")
        if self.timer_period <= 0.0:
            raise ValueError("timer_period must be positive")
        if self.max_pending_scans < 1:
            raise ValueError("max_pending_scans must be at least 1")
        if self.decay_time >= 8.0:
            rospy.logwarn(
                "[scan_decay_gate] decay_time %.1fs is close to the usual "
                "10s TF cache; delayed clearing scans may be dropped.",
                self.decay_time,
            )

        self.lock = threading.RLock()
        self.enabled = False
        self.pending_scans = collections.deque()

        self.scan_publisher = rospy.Publisher(
            self.output_topic, LaserScan, queue_size=1
        )
        self.clearing_publisher = rospy.Publisher(
            self.clearing_topic, LaserScan, queue_size=10
        )
        self.enable_subscriber = rospy.Subscriber(
            self.enable_topic, Bool, self.enable_callback, queue_size=1
        )
        self.scan_subscriber = rospy.Subscriber(
            self.input_topic, LaserScan, self.scan_callback, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(self.timer_period), self.timer_callback
        )

        rospy.logwarn(
            "[scan_decay_gate] LOCKED until %s=true; %s -> %s, old marks "
            "clear through %s after %.1fs.",
            self.enable_topic,
            self.input_topic,
            self.output_topic,
            self.clearing_topic,
            self.decay_time,
        )

    def enable_callback(self, message):
        enabled = bool(message.data)
        with self.lock:
            if enabled == self.enabled:
                return
            self.enabled = enabled
            # A localization reset clears the layers.  Never replay scans from
            # the pre-reset TF epoch after the guard enables input again.
            self.pending_scans.clear()

        if enabled:
            rospy.loginfo("[scan_decay_gate] Live costmap scan ENABLED.")
        else:
            rospy.logerr(
                "[scan_decay_gate] Live costmap scan LOCKED; pending decay "
                "history discarded."
            )

    def scan_callback(self, message):
        with self.lock:
            if not self.enabled:
                rospy.logwarn_throttle(
                    2.0,
                    "[scan_decay_gate] Withholding raw scan until localization "
                    "is stable.",
                )
                return

            self.scan_publisher.publish(message)
            self.pending_scans.append((time.monotonic(), message))
            while len(self.pending_scans) > self.max_pending_scans:
                # Failing to clear is worse than clearing slightly early.
                _, oldest = self.pending_scans.popleft()
                self.clearing_publisher.publish(self.make_clearing_scan(oldest))

    def timer_callback(self, _event):
        now = time.monotonic()
        expired = []
        with self.lock:
            if not self.enabled:
                return
            while (
                self.pending_scans
                and now - self.pending_scans[0][0] >= self.decay_time
            ):
                _, message = self.pending_scans.popleft()
                expired.append(self.make_clearing_scan(message))

        for message in expired:
            self.clearing_publisher.publish(message)

        if expired:
            rospy.logdebug(
                "[scan_decay_gate] Published %d expired scan(s) for clearing.",
                len(expired),
            )

    def make_clearing_scan(self, message):
        """Extend old finite endpoints past cells shortened by VoxelLayer."""
        clearing = copy.deepcopy(message)
        maximum = max(message.range_min, message.range_max - 1.0e-3)
        clearing.ranges = [
            min(value + self.clear_extension, maximum)
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
            else value
            for value in message.ranges
        ]
        return clearing


if __name__ == "__main__":
    rospy.init_node("scan_decay_gate_obstacle")
    ScanDecayGateObstacle()
    rospy.spin()

