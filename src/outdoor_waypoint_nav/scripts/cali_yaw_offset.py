#!/usr/bin/env python3
"""Calibrate robot_localization's navsat yaw_offset from a straight RTK run.

The yaw source is an IMU topic.  GNSS heading is intentionally not used.  A
NavSatFix topic is used only to estimate the true ENU direction of travel.
After a valid 10--15 m straight run, exactly the yaw_offset scalar in the
configured YAML file is replaced atomically.
"""

import json
import math
import os
import re
import stat
import statistics
import tempfile
import threading
from collections import deque

import rospy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String


EARTH_RADIUS_M = 6378137.0
IMU_TOPIC = "/imu/data_new"


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("IMU quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("IMU quaternion has zero norm")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def circular_mean(values):
    values = list(values)
    if not values:
        raise ValueError("cannot average an empty angle sequence")
    return math.atan2(
        statistics.fmean(math.sin(value) for value in values),
        statistics.fmean(math.cos(value) for value in values),
    )


def circular_rms(values, center):
    values = list(values)
    if not values:
        return 0.0
    return math.sqrt(
        statistics.fmean(normalize_angle(value - center) ** 2 for value in values)
    )


def fit_line_once(samples):
    """Orthogonal 2-D line fit. Samples contain x/y in fields 0 and 1."""
    if len(samples) < 2:
        raise ValueError("at least two samples are required for a line fit")

    mean_x = statistics.fmean(sample[0] for sample in samples)
    mean_y = statistics.fmean(sample[1] for sample in samples)
    covariance_xx = statistics.fmean((sample[0] - mean_x) ** 2 for sample in samples)
    covariance_yy = statistics.fmean((sample[1] - mean_y) ** 2 for sample in samples)
    covariance_xy = statistics.fmean(
        (sample[0] - mean_x) * (sample[1] - mean_y) for sample in samples
    )

    course = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy,
    )
    delta_x = samples[-1][0] - samples[0][0]
    delta_y = samples[-1][1] - samples[0][1]
    if math.cos(course) * delta_x + math.sin(course) * delta_y < 0.0:
        course = normalize_angle(course + math.pi)

    normal_x = -math.sin(course)
    normal_y = math.cos(course)
    residuals = [
        abs((sample[0] - mean_x) * normal_x + (sample[1] - mean_y) * normal_y)
        for sample in samples
    ]
    return course, residuals


def robust_line_fit(samples, absolute_outlier_m):
    inliers = list(samples)
    for _ in range(4):
        course, residuals = fit_line_once(inliers)
        median = statistics.median(residuals)
        mad = statistics.median(abs(value - median) for value in residuals)
        robust_limit = median + 3.5 * max(1.4826 * mad, 0.02)
        limit = max(float(absolute_outlier_m), robust_limit)
        filtered = [
            sample for sample, residual in zip(inliers, residuals) if residual <= limit
        ]
        if len(filtered) < 2 or len(filtered) == len(inliers):
            break
        inliers = filtered

    course, residuals = fit_line_once(inliers)
    return {
        "course": course,
        "inliers": inliers,
        "rms_m": math.sqrt(statistics.fmean(value * value for value in residuals)),
        "max_m": max(residuals),
    }


def read_yaml_scalar(path, key):
    with open(path, "r", encoding="utf-8") as stream:
        text = stream.read()
    pattern = re.compile(
        r"^(\s*" + re.escape(key) + r"\s*:\s*)([^#\r\n]+?)(\s*(?:#.*)?)$",
        re.MULTILINE,
    )
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError(
            "%s must contain exactly one '%s:' entry; found %d"
            % (path, key, len(matches))
        )
    return float(matches[0].group(2).strip())


def replace_yaml_scalar_atomic(path, key, value_text):
    """Replace one YAML scalar while preserving every other byte-level line."""
    with open(path, "r", encoding="utf-8") as stream:
        original = stream.read()

    pattern = re.compile(
        r"^(\s*" + re.escape(key) + r"\s*:\s*)([^#\r\n]+?)(\s*(?:#.*)?)$",
        re.MULTILINE,
    )
    matches = list(pattern.finditer(original))
    if len(matches) != 1:
        raise ValueError(
            "%s must contain exactly one '%s:' entry; found %d"
            % (path, key, len(matches))
        )

    old_value = matches[0].group(2).strip()
    updated = pattern.sub(
        lambda match: match.group(1) + value_text + match.group(3),
        original,
        count=1,
    )
    if updated == original:
        return old_value, False

    directory = os.path.dirname(os.path.abspath(path))
    source_mode = stat.S_IMODE(os.stat(path).st_mode)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".yaw_offset.", suffix=".tmp", dir=directory
    )
    try:
        os.fchmod(descriptor, source_mode)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            descriptor = -1
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except Exception:
        if descriptor >= 0:
            os.close(descriptor)
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise

    return old_value, True


class YawOffsetCalibrator:
    def __init__(self):
        self.lock = threading.RLock()
        self.done = False
        self.start_time = rospy.Time.now()

        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")
        # This calibration must use exactly the same dedicated INS heading as
        # navsat_transform and the EKFs. Keep it constant so a stale ROS
        # parameter cannot silently switch the calibration to another IMU.
        self.imu_topic = IMU_TOPIC
        self.config_file = os.path.abspath(
            os.path.expanduser(rospy.get_param("~config_file"))
        )

        self.max_gps_variance = float(rospy.get_param("~max_gps_variance_m2", 0.04))
        self.sync_slop = float(rospy.get_param("~sync_slop_sec", 0.25))
        self.baseline_samples_required = int(rospy.get_param("~baseline_samples", 8))
        self.max_baseline_radius = float(rospy.get_param("~max_baseline_radius_m", 0.20))
        self.start_distance = float(rospy.get_param("~start_distance_m", 0.40))
        self.min_distance = float(rospy.get_param("~min_distance_m", 10.0))
        self.max_distance = float(rospy.get_param("~max_distance_m", 15.0))
        self.stop_duration = float(rospy.get_param("~stop_duration_sec", 2.0))
        self.stop_radius = float(rospy.get_param("~stop_radius_m", 0.18))
        self.min_samples = int(rospy.get_param("~min_samples", 20))
        self.outlier_threshold = float(rospy.get_param("~outlier_threshold_m", 0.50))
        self.max_line_rms = float(rospy.get_param("~max_line_rms_m", 0.25))
        self.max_line_error = float(rospy.get_param("~max_line_error_m", 0.60))
        self.max_course_difference = math.radians(
            float(rospy.get_param("~max_course_difference_deg", 2.0))
        )
        self.max_imu_yaw_rms = math.radians(
            float(rospy.get_param("~max_imu_yaw_rms_deg", 5.0))
        )
        self.min_imu_messages = int(rospy.get_param("~min_imu_messages", 30))
        self.min_unique_imu_values = int(rospy.get_param("~min_unique_imu_values", 5))
        self.output_decimals = int(rospy.get_param("~output_decimals", 3))
        self.max_wait = float(rospy.get_param("~max_wait_sec", 300.0))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        if not os.path.isfile(self.config_file):
            raise ValueError("config file does not exist: %s" % self.config_file)
        if not self.dry_run and not os.access(self.config_file, os.W_OK):
            raise ValueError("config file is not writable: %s" % self.config_file)
        if self.baseline_samples_required < 3:
            raise ValueError("baseline_samples must be at least 3")
        if not 0.0 < self.min_distance < self.max_distance:
            raise ValueError("require 0 < min_distance_m < max_distance_m")
        if self.output_decimals < 0 or self.output_decimals > 9:
            raise ValueError("output_decimals must be in the range 0..9")

        self.magnetic_declination = self._resolve_magnetic_declination()
        self.origin = None
        self.latest_imu = None
        self.baseline = deque(maxlen=self.baseline_samples_required)
        self.baseline_xy = None
        self.armed = False
        self.moving = False
        self.start_xy = None
        self.samples = []
        self.recent = deque()
        self.next_progress = 1.0
        self.imu_message_count = 0
        self.imu_orientation_fingerprints = set()

        self.result_publisher = rospy.Publisher(
            "~result", Float64, queue_size=1, latch=True
        )
        self.status_publisher = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.imu_topic, Imu, self._imu_callback, queue_size=200)
        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_callback, queue_size=50)
        rospy.Timer(rospy.Duration(1.0), self._timer_callback)

        self._publish_status("WAITING_FOR_STATIONARY_BASELINE")
        rospy.logwarn(
            "[yaw_cal] Keep the robot stationary until ARMED, then drive straight %.1f--%.1f m and stop.",
            self.min_distance,
            self.max_distance,
        )
        rospy.loginfo(
            "[yaw_cal] heading_source=%s gps_position=%s gps_heading=UNUSED config=%s declination=%.7f rad",
            self.imu_topic,
            self.gps_topic,
            self.config_file,
            self.magnetic_declination,
        )

    def _resolve_magnetic_declination(self):
        private_value = rospy.get_param("~magnetic_declination_radians", None)
        if private_value is not None:
            return float(private_value)
        live_name = "/outdoor_waypoint_nav/navsat_transform/magnetic_declination_radians"
        if rospy.has_param(live_name):
            return float(rospy.get_param(live_name))
        return read_yaml_scalar(self.config_file, "magnetic_declination_radians")

    @staticmethod
    def _stamp(header):
        return header.stamp if header.stamp != rospy.Time(0) else rospy.Time.now()

    @staticmethod
    def _imu_orientation_fingerprint(msg):
        values = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        return tuple(round(float(value), 10) for value in values)

    def _imu_callback(self, msg):
        try:
            yaw = quaternion_yaw(msg.orientation)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "[yaw_cal] Invalid IMU orientation: %s", exc)
            return

        stamp = self._stamp(msg.header)
        with self.lock:
            self.latest_imu = (stamp, yaw)
            if self.moving and not self.done:
                self.imu_message_count += 1
                if len(self.imu_orientation_fingerprints) < 10000:
                    self.imu_orientation_fingerprints.add(
                        self._imu_orientation_fingerprint(msg)
                    )

    def _local_xy(self, latitude, longitude):
        latitude_rad = math.radians(latitude)
        longitude_rad = math.radians(longitude)
        if self.origin is None:
            self.origin = (latitude_rad, longitude_rad)
        origin_latitude, origin_longitude = self.origin
        east = (
            EARTH_RADIUS_M
            * math.cos(0.5 * (latitude_rad + origin_latitude))
            * (longitude_rad - origin_longitude)
        )
        north = EARTH_RADIUS_M * (latitude_rad - origin_latitude)
        return east, north

    def _gps_variance(self, msg):
        covariance = msg.position_covariance
        if len(covariance) < 5:
            return float("inf")
        values = (float(covariance[0]), float(covariance[4]))
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            return float("inf")
        return max(values)

    def _gps_callback(self, msg):
        if self.done:
            return
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            rospy.logwarn_throttle(5.0, "[yaw_cal] Waiting for a valid GNSS fix")
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        gps_variance = self._gps_variance(msg)
        if gps_variance > self.max_gps_variance:
            rospy.logwarn_throttle(
                5.0,
                "[yaw_cal] RTK quality rejected: variance %.4f > %.4f m^2",
                gps_variance,
                self.max_gps_variance,
            )
            return

        gps_stamp = self._stamp(msg.header)
        with self.lock:
            imu_item = self.latest_imu
        if imu_item is None:
            rospy.logwarn_throttle(5.0, "[yaw_cal] Waiting for IMU on %s", self.imu_topic)
            return
        imu_stamp, imu_yaw = imu_item
        age = abs((gps_stamp - imu_stamp).to_sec())
        if age > self.sync_slop:
            rospy.logwarn_throttle(
                5.0,
                "[yaw_cal] GPS/IMU timestamps differ by %.3f s (limit %.3f s)",
                age,
                self.sync_slop,
            )
            return

        x, y = self._local_xy(msg.latitude, msg.longitude)
        sample = (x, y, gps_stamp.to_sec(), imu_yaw, gps_variance)

        with self.lock:
            if self.done:
                return
            if not self.armed:
                self.baseline.append(sample)
                if len(self.baseline) < self.baseline_samples_required:
                    return
                mean_x = statistics.fmean(item[0] for item in self.baseline)
                mean_y = statistics.fmean(item[1] for item in self.baseline)
                radius = max(
                    math.hypot(item[0] - mean_x, item[1] - mean_y)
                    for item in self.baseline
                )
                if radius > self.max_baseline_radius:
                    rospy.logwarn_throttle(
                        2.0,
                        "[yaw_cal] Robot/GPS is not stationary: baseline radius %.3f m",
                        radius,
                    )
                    return
                self.baseline_xy = (mean_x, mean_y)
                self.armed = True
                self._publish_status("ARMED")
                rospy.logwarn(
                    "[yaw_cal] ARMED. Drive STRAIGHT %.1f--%.1f m, then stop for %.1f s.",
                    self.min_distance,
                    self.max_distance,
                    self.stop_duration,
                )
                return

            if not self.moving:
                distance_from_start = math.hypot(
                    x - self.baseline_xy[0], y - self.baseline_xy[1]
                )
                if distance_from_start < self.start_distance:
                    return
                baseline_yaw = circular_mean(item[3] for item in self.baseline)
                baseline_variance = statistics.fmean(item[4] for item in self.baseline)
                self.start_xy = self.baseline_xy
                self.samples = [
                    (
                        self.start_xy[0],
                        self.start_xy[1],
                        self.baseline[-1][2],
                        baseline_yaw,
                        baseline_variance,
                    )
                ]
                self.recent.clear()
                self.imu_message_count = 0
                self.imu_orientation_fingerprints.clear()
                self.moving = True
                self._publish_status("MEASURING")
                rospy.logwarn("[yaw_cal] Motion detected; calibration measurement started")

            self.samples.append(sample)
            self.recent.append(sample)
            # Retain the sample immediately before the stop-duration boundary
            # plus every newer sample. This gives a full time window without
            # unnecessarily making a requested 2 s stop take about 3 s.
            while (
                len(self.recent) >= 2
                and sample[2] - self.recent[1][2] >= self.stop_duration
            ):
                self.recent.popleft()

            distance = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
            if distance >= self.next_progress:
                rospy.loginfo(
                    "[yaw_cal] progress %.2f m / %.1f--%.1f m (%d GPS samples)",
                    distance,
                    self.min_distance,
                    self.max_distance,
                    len(self.samples),
                )
                self.next_progress += 1.0

            if distance >= self.max_distance:
                rospy.logwarn(
                    "[yaw_cal] Reached %.2f m. STOP NOW; calculating from the first %.1f m.",
                    distance,
                    self.max_distance,
                )
                samples = list(self.samples)
            elif distance >= self.min_distance and self._is_stationary(sample[2]):
                rospy.logwarn(
                    "[yaw_cal] Stop detected at %.2f m; validating calibration.", distance
                )
                samples = list(self.samples)
            else:
                return

            self.done = True

        self._calculate_and_write(samples)

    def _is_stationary(self, current_time):
        if len(self.recent) < 4:
            return False
        if current_time - self.recent[0][2] < self.stop_duration:
            return False
        mean_x = statistics.fmean(item[0] for item in self.recent)
        mean_y = statistics.fmean(item[1] for item in self.recent)
        radius = max(
            math.hypot(item[0] - mean_x, item[1] - mean_y) for item in self.recent
        )
        return radius <= self.stop_radius

    def _calculate_and_write(self, samples):
        try:
            result = self._calculate(samples)
            rounded_text = ("%%.%df" % self.output_decimals) % result["yaw_offset_rad"]
            result["written_yaw_offset"] = float(rounded_text)
            result["config_file"] = self.config_file

            if self.dry_run:
                changed = False
                old_value = read_yaml_scalar(self.config_file, "yaw_offset")
                rospy.logwarn(
                    "[yaw_cal] DRY RUN: would replace yaw_offset %s -> %s",
                    old_value,
                    rounded_text,
                )
            else:
                old_value, changed = replace_yaml_scalar_atomic(
                    self.config_file, "yaw_offset", rounded_text
                )
                rospy.logwarn(
                    "[yaw_cal] SUCCESS: yaw_offset %s -> %s in %s",
                    old_value,
                    rounded_text,
                    self.config_file,
                )
            result["previous_yaw_offset"] = float(old_value)
            result["file_changed"] = bool(changed)

            payload = json.dumps(result, sort_keys=True)
            self.result_publisher.publish(Float64(data=float(rounded_text)))
            self._publish_status("SUCCESS " + payload)
            rospy.loginfo("[yaw_cal] Result: %s", payload)
            rospy.logwarn(
                "[yaw_cal] Restart the two navigation launch files before testing; "
                "the running navsat_transform will not reload this file."
            )
            rospy.signal_shutdown("yaw offset calibration complete")
        except Exception as exc:
            self._fail(str(exc))

    def _calculate(self, samples):
        if len(samples) < self.min_samples:
            raise ValueError(
                "only %d GPS samples were collected; need at least %d"
                % (len(samples), self.min_samples)
            )
        distance = math.hypot(
            samples[-1][0] - samples[0][0],
            samples[-1][1] - samples[0][1],
        )
        if distance < self.min_distance:
            raise ValueError(
                "straight-line displacement %.2f m is below %.2f m"
                % (distance, self.min_distance)
            )

        fit = robust_line_fit(samples, self.outlier_threshold)
        inliers = fit["inliers"]
        if len(inliers) < self.min_samples:
            raise ValueError(
                "only %d inliers remain after GPS outlier rejection" % len(inliers)
            )
        if fit["rms_m"] > self.max_line_rms:
            raise ValueError(
                "path is not straight enough: RMS %.3f m > %.3f m"
                % (fit["rms_m"], self.max_line_rms)
            )
        if fit["max_m"] > self.max_line_error:
            raise ValueError(
                "path has a %.3f m lateral error > %.3f m"
                % (fit["max_m"], self.max_line_error)
            )

        endpoint_course = math.atan2(
            samples[-1][1] - samples[0][1],
            samples[-1][0] - samples[0][0],
        )
        course_difference = abs(normalize_angle(fit["course"] - endpoint_course))
        if course_difference > self.max_course_difference:
            raise ValueError(
                "line-fit and endpoint courses differ by %.2f deg > %.2f deg"
                % (
                    math.degrees(course_difference),
                    math.degrees(self.max_course_difference),
                )
            )

        mean_imu_yaw = circular_mean(sample[3] for sample in inliers)
        imu_yaw_rms = circular_rms((sample[3] for sample in inliers), mean_imu_yaw)
        if imu_yaw_rms > self.max_imu_yaw_rms:
            raise ValueError(
                "IMU yaw varied by RMS %.2f deg; drive straighter (limit %.2f deg)"
                % (
                    math.degrees(imu_yaw_rms),
                    math.degrees(self.max_imu_yaw_rms),
                )
            )

        if self.imu_message_count < self.min_imu_messages:
            raise ValueError(
                "only %d IMU messages arrived during motion; need at least %d"
                % (self.imu_message_count, self.min_imu_messages)
            )
        unique_imu_values = len(self.imu_orientation_fingerprints)
        if unique_imu_values < self.min_unique_imu_values:
            raise ValueError(
                "IMU orientation appears frozen: %d messages but only %d distinct "
                "quaternions. "
                "Do not calibrate yaw_offset from this topic."
                % (self.imu_message_count, unique_imu_values)
            )

        # The fitted course is true local ENU. navsat_transform adds magnetic
        # declination + yaw_offset + UTM convergence to the IMU yaw. The same
        # convergence rotates true ENU course into UTM grid course, so it
        # cancels here and must not be subtracted a second time.
        total_imu_correction = normalize_angle(fit["course"] - mean_imu_yaw)
        yaw_offset = normalize_angle(
            total_imu_correction - self.magnetic_declination
        )

        return {
            "gps_topic": self.gps_topic,
            "imu_topic": self.imu_topic,
            "gps_heading_used": False,
            "sample_count": len(samples),
            "inlier_count": len(inliers),
            "distance_m": distance,
            "course_fit_deg_enu": math.degrees(fit["course"]),
            "course_endpoint_deg_enu": math.degrees(endpoint_course),
            "course_difference_deg": math.degrees(course_difference),
            "mean_imu_yaw_deg": math.degrees(mean_imu_yaw),
            "imu_yaw_rms_deg": math.degrees(imu_yaw_rms),
            "imu_messages": self.imu_message_count,
            "unique_imu_values": unique_imu_values,
            "line_rms_m": fit["rms_m"],
            "line_max_error_m": fit["max_m"],
            "magnetic_declination_rad": self.magnetic_declination,
            "total_imu_correction_rad": total_imu_correction,
            "yaw_offset_rad": yaw_offset,
            "yaw_offset_deg": math.degrees(yaw_offset),
        }

    def _timer_callback(self, _event):
        if self.done or self.max_wait <= 0.0:
            return
        if (rospy.Time.now() - self.start_time).to_sec() > self.max_wait:
            with self.lock:
                if self.done:
                    return
                self.done = True
            self._fail("timed out after %.0f seconds; YAML was not changed" % self.max_wait)

    def _publish_status(self, text):
        self.status_publisher.publish(String(data=text))

    def _fail(self, reason):
        self._publish_status("FAILED " + reason)
        rospy.logerr("[yaw_cal] FAILED: %s", reason)
        rospy.logerr("[yaw_cal] navsat_params.yaml was NOT changed")
        rospy.signal_shutdown("yaw offset calibration failed")


def main():
    rospy.init_node("cali_yaw_offset")
    try:
        YawOffsetCalibrator()
    except Exception as exc:
        rospy.logfatal("[yaw_cal] Cannot start: %s", exc)
        return
    rospy.spin()


if __name__ == "__main__":
    main()
