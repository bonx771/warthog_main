#!/usr/bin/env python3
"""Waypoint collector that accepts every valid GNSS positioning mode.

The legacy obstacle collector intentionally requires RTK FIXED.  This entry
point reuses its sampling, TF validation, marker publishing, and file handling,
while replacing the input-quality policy.  Every saved coordinate is calculated
only from valid raw GNSS fixes received during the five-second sampling window.
Outliers are removed in metric UTM space, then the remaining samples are
averaged with their reported covariance.  The same raw-GNSS result is written
to the waypoint file and is therefore also used when a goal is sent.  NO_FIX is
still rejected and a stale fix can never be saved.
"""

import math
import statistics

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus

import filtered_collect_gps_waypoints as legacy


def raw_arithmetic_mean(values):
    values = list(values)
    if not values:
        raise ValueError("cannot calculate a raw GNSS mean without samples")
    return math.fsum(values) / float(len(values))


def robust_raw_gnss_mean(
    samples,
    trim_fraction=0.20,
    covariance_floor_m2=0.04,
    minimum_inliers=5,
):
    """Return a covariance-weighted mean after rejecting radial UTM outliers.

    Every input and the returned latitude/longitude remain GNSS measurements;
    no base_link pose is used.  Working in UTM avoids treating degrees as a
    metric coordinate, and radial trimming prevents one receiver jump from
    dragging the saved waypoint several metres away.
    """
    if not samples:
        raise ValueError("cannot filter an empty raw GNSS sample window")

    trim_fraction = min(0.45, max(0.0, float(trim_fraction)))
    covariance_floor_m2 = max(1.0e-6, float(covariance_floor_m2))

    records = []
    expected_zone = None
    for sample in samples:
        easting, northing, zone_number, zone_letter = legacy.utm.from_latlon(
            sample.latitude,
            sample.longitude,
        )
        zone = (zone_number, zone_letter)
        if expected_zone is None:
            expected_zone = zone
        elif zone != expected_zone:
            raise ValueError(
                "raw GNSS samples cross UTM zones: {} and {}".format(
                    expected_zone,
                    zone,
                )
            )
        records.append((sample, easting, northing))

    median_easting = statistics.median(record[1] for record in records)
    median_northing = statistics.median(record[2] for record in records)
    ranked = sorted(
        records,
        key=lambda record: math.hypot(
            record[1] - median_easting,
            record[2] - median_northing,
        ),
    )

    requested_inliers = int(math.ceil(len(ranked) * (1.0 - trim_fraction)))
    keep_count = min(
        len(ranked),
        max(1, int(minimum_inliers), requested_inliers),
    )
    inliers = ranked[:keep_count]

    east_weight_sum = 0.0
    north_weight_sum = 0.0
    weighted_easting_sum = 0.0
    weighted_northing_sum = 0.0
    for sample, easting, northing in inliers:
        east_variance = max(
            covariance_floor_m2,
            float(sample.position_covariance[0]),
        )
        north_variance = max(
            covariance_floor_m2,
            float(sample.position_covariance[4]),
        )
        east_weight = 1.0 / east_variance
        north_weight = 1.0 / north_variance
        weighted_easting_sum += east_weight * easting
        weighted_northing_sum += north_weight * northing
        east_weight_sum += east_weight
        north_weight_sum += north_weight

    mean_easting = weighted_easting_sum / east_weight_sum
    mean_northing = weighted_northing_sum / north_weight_sum
    mean_latitude, mean_longitude = legacy.utm.to_latlon(
        mean_easting,
        mean_northing,
        expected_zone[0],
        expected_zone[1],
    )
    return (
        mean_latitude,
        mean_longitude,
        mean_easting,
        mean_northing,
        inliers,
    )


def total_fix_rejection_reason(msg, max_horizontal_stddev):
    if msg.status.status < NavSatStatus.STATUS_FIX:
        return "GNSS reports NO_FIX"

    if (
        not math.isfinite(msg.latitude)
        or not math.isfinite(msg.longitude)
        or not -90.0 <= msg.latitude <= 90.0
        or not -180.0 <= msg.longitude <= 180.0
    ):
        return "latitude/longitude is invalid"

    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return "horizontal covariance is unknown"

    covariance_x = float(msg.position_covariance[0])
    covariance_y = float(msg.position_covariance[4])
    if (
        not math.isfinite(covariance_x)
        or not math.isfinite(covariance_y)
        or covariance_x < 0.0
        or covariance_y < 0.0
    ):
        return "horizontal covariance is invalid"

    horizontal_stddev = math.sqrt(max(covariance_x, covariance_y))
    if max_horizontal_stddev > 0.0 and horizontal_stddev > max_horizontal_stddev:
        return "horizontal sigma %.3f m exceeds %.3f m" % (
            horizontal_stddev,
            max_horizontal_stddev,
        )

    return ""


class TotalWaypointCollector(legacy.FilteredWaypointCollector):
    @staticmethod
    def adaptive_projection_limit(
        minimum_limit_m,
        sigma_multiplier,
        maximum_limit_m,
        horizontal_sigma_m,
    ):
        limit = max(minimum_limit_m, sigma_multiplier * horizontal_sigma_m)
        if maximum_limit_m > 0.0:
            limit = min(limit, maximum_limit_m)
        return limit

    def print_startup_logs(self):
        rospy.loginfo("Initiated total_collect_gps_waypoints node")
        rospy.loginfo("Collecting waypoints from GNSS topic: %s", self.gps_topic)
        rospy.loginfo("Waypoint sampling window: %.1f s", self.sample_duration)
        rospy.loginfo("Minimum samples required per waypoint: %d", self.min_samples)
        trim_fraction = min(
            0.45,
            max(
                0.0,
                float(
                    rospy.get_param(
                        "/outdoor_waypoint_nav/waypoint_outlier_trim_fraction",
                        0.20,
                    )
                ),
            ),
        )
        rospy.loginfo(
            "Saved waypoint position: robust covariance-weighted mean of raw GNSS samples"
        )
        rospy.loginfo(
            "Raw GNSS outlier rejection: farthest %.0f%% of the 5 s window",
            100.0 * trim_fraction,
        )
        if self.rtk_max_horizontal_stddev > 0.0:
            rospy.loginfo(
                "GNSS collection policy: any valid fix, horizontal sigma <= %.3f m, age <= %.2f s",
                self.rtk_max_horizontal_stddev,
                self.rtk_fix_timeout,
            )
        else:
            rospy.loginfo(
                "GNSS collection policy: any valid fix with valid covariance, age <= %.2f s",
                self.rtk_fix_timeout,
            )
        if self.require_navigation_ready:
            rospy.loginfo(
                "Waypoint collection waits for navigation-ready topic: %s",
                self.navigation_ready_topic,
            )
        rospy.loginfo("Saving filtered coordinates to: %s", self.coordinates_path)
        rospy.loginfo("Publishing collected waypoint markers to: %s", self.marker_topic)
        rospy.logwarn(
            "Waiting for a valid GPS NORMAL/RTK FLOAT/RTK FIXED solution and stable localization."
        )
        print("")
        print("Press {} button to collect and store waypoint.".format(self.collect_button_sym))
        print("Press {} button to end waypoint collection.".format(self.end_button_sym))
        print("")

    def gps_callback(self, msg):
        rejection_reason = total_fix_rejection_reason(
            msg, self.rtk_max_horizontal_stddev
        )
        if rejection_reason:
            rospy.logwarn_throttle(
                2.0,
                "GNSS fix is not usable on %s: %s. Waiting for a valid fix.",
                self.gps_topic,
                rejection_reason,
            )
            self.have_latest_fix = False
            self.latest_fix_rejection_reason = rejection_reason
            return

        self.latest_fix = msg
        self.have_latest_fix = True
        self.latest_fix_received_at = rospy.Time.now()
        self.latest_fix_rejection_reason = ""
        if self.sampling_active:
            self.samples.append(msg)

        if not self.first_valid_fix_logged:
            rospy.loginfo(
                "Received usable GNSS point on %s: lat=%.8f lon=%.8f sigma=%.3f m status=%d",
                self.gps_topic,
                msg.latitude,
                msg.longitude,
                legacy.horizontal_stddev(msg),
                msg.status.status,
            )
            self.first_valid_fix_logged = True

    def start_sampling(self):
        if self.sampling_active:
            remaining = max(0.0, (self.sample_end - rospy.Time.now()).to_sec())
            rospy.logwarn(
                "Waypoint sampling is already active. Keep robot stopped for %.1f s more.",
                remaining,
            )
            return

        if self.require_navigation_ready and not self.navigation_ready:
            rospy.logerr(
                "Cannot sample waypoint: localization is not ready on %s. "
                "Keep the UGV stopped and wait for NAVIGATION READY.",
                self.navigation_ready_topic,
            )
            return

        fix_usable, rejection_reason = self.latest_rtk_fix_is_fresh()
        if not fix_usable:
            rospy.logerr(
                "Cannot sample waypoint: no fresh valid GNSS fix on %s (%s).",
                self.gps_topic,
                rejection_reason,
            )
            return

        self.samples = []
        self.sampling_localization_invalidated = False
        self.sample_start = rospy.Time.now()
        self.sample_end = self.sample_start + rospy.Duration(self.sample_duration)
        self.sampling_active = True

        rospy.logwarn(
            "Waypoint collect button pressed. Keep robot STOPPED and wait %.1f s before moving.",
            self.sample_duration,
        )
        rospy.loginfo(
            "Averaging raw waypoint GNSS now. Do not move until the saved-waypoint log appears."
        )

    def save_filtered_waypoint(self):
        """Save a robust mean of the five-second raw-GNSS window."""
        self.sampling_active = False

        if self.sampling_localization_invalidated:
            rospy.logerr(
                "Waypoint not saved: localization lost readiness during the sampling window. "
                "Keep the UGV stopped, wait for NAVIGATION READY, then collect again."
            )
            return

        if self.require_navigation_ready and not self.navigation_ready:
            rospy.logerr(
                "Waypoint not saved: localization is no longer ready. "
                "Keep the UGV stopped, wait for NAVIGATION READY, then collect again."
            )
            return

        fix_usable, rejection_reason = self.latest_rtk_fix_is_fresh()
        if not fix_usable:
            rospy.logerr(
                "Waypoint not saved: raw GNSS fix was lost or became stale (%s).",
                rejection_reason,
            )
            rospy.logwarn(
                "You may move now, but collect this waypoint again after GNSS is stable."
            )
            return

        if len(self.samples) < self.min_samples:
            rospy.logerr(
                "Waypoint not saved: only %d raw GPS sample(s) in %.1f s, need at least %d.",
                len(self.samples),
                self.sample_duration,
                self.min_samples,
            )
            rospy.logwarn(
                "You may move now, but collect this waypoint again after GPS is stable."
            )
            return

        try:
            trim_fraction = min(
                0.45,
                max(
                    0.0,
                    float(
                        rospy.get_param(
                            "/outdoor_waypoint_nav/waypoint_outlier_trim_fraction",
                            0.20,
                        )
                    ),
                ),
            )
            covariance_floor_m2 = max(
                1.0e-6,
                float(
                    rospy.get_param(
                        "/outdoor_waypoint_nav/waypoint_covariance_weight_floor_m2",
                        0.04,
                    )
                )
            )
            (
                mean_latitude,
                mean_longitude,
                center_easting,
                center_northing,
                inliers,
            ) = robust_raw_gnss_mean(
                self.samples,
                trim_fraction=trim_fraction,
                covariance_floor_m2=covariance_floor_m2,
                minimum_inliers=self.min_samples,
            )
            distances = [
                math.hypot(
                    record[1] - center_easting,
                    record[2] - center_northing,
                )
                for record in inliers
            ]
        except Exception as exc:
            rospy.logerr(
                "Waypoint not saved: unable to robust-filter raw GPS in UTM: %s",
                exc,
            )
            rospy.logwarn(
                "You may move now, but collect this waypoint again after GPS is stable."
            )
            return

        rms_spread = math.sqrt(
            sum(distance * distance for distance in distances) / len(distances)
        )
        max_spread = max(distances)
        max_horizontal_sigma = max(
            legacy.horizontal_stddev(record[0]) for record in inliers
        )

        if self.last_utm is not None:
            distance_from_last = math.hypot(
                center_easting - self.last_utm[0],
                center_northing - self.last_utm[1],
            )
            if distance_from_last < self.min_waypoint_distance_m:
                rospy.logwarn(
                    "Raw-GPS waypoint not saved: %.2f m from previous waypoint, need %.2f m.",
                    distance_from_last,
                    self.min_waypoint_distance_m,
                )
                rospy.logwarn(
                    "You may move now, but move farther before collecting the next waypoint."
                )
                return

        next_waypoint_index = self.waypoint_count + 1
        marker_point = self.validate_projected_waypoint(
            center_easting,
            center_northing,
            next_waypoint_index,
        )
        if marker_point is None:
            rospy.logwarn(
                "Waypoint was NOT written to %s. Keep the UGV stopped, wait for stable localization, then collect again.",
                self.coordinates_path,
            )
            return

        try:
            self.coord_file.write(
                "{:.8f} {:.8f}\n".format(mean_latitude, mean_longitude)
            )
            self.coord_file.flush()
        except IOError as exc:
            rospy.logerr(
                "Waypoint not saved: cannot write %s: %s",
                self.coordinates_path,
                exc,
            )
            return

        self.waypoint_count = next_waypoint_index
        self.last_utm = (center_easting, center_northing)
        self.publish_waypoint_marker(marker_point, self.waypoint_count)

        rospy.loginfo(
            "Collected robust raw-GNSS waypoint %d: lat=%.8f lon=%.8f "
            "samples=%d inliers=%d rejected=%d window=%.1f s "
            "rms=%.2f m max=%.2f m max_sigma=%.3f m",
            self.waypoint_count,
            mean_latitude,
            mean_longitude,
            len(self.samples),
            len(inliers),
            len(self.samples) - len(inliers),
            self.sample_duration,
            rms_spread,
            max_spread,
            max_horizontal_sigma,
        )
        rospy.logwarn("Raw-GNSS waypoint saved. You may move now.")
        rospy.loginfo(
            "Press %s button to collect next waypoint %d.",
            self.collect_button_sym,
            self.waypoint_count + 1,
        )
        rospy.loginfo(
            "Press %s button to end waypoint collection.",
            self.end_button_sym,
        )

    def validate_projected_waypoint(self, easting, northing, waypoint_index):
        """Use a covariance-aware GPS/map consistency gate.

        The legacy collector uses one fixed distance because it only accepts
        centimetre-level RTK FIXED.  For this total collector, a normal GPS fix
        with sigma=2 m legitimately needs a larger gate than RTK FLOAT/FIXED.
        """
        marker_point = self.transform_utm_point(easting, northing, self.marker_frame)
        if marker_point is None:
            rospy.logerr(
                "Waypoint %d rejected: cannot transform GNSS point from utm into %s.",
                waypoint_index,
                self.marker_frame,
            )
            return None

        sigma_multiplier = max(
            0.0,
            float(
                rospy.get_param(
                    "/outdoor_waypoint_nav/waypoint_projection_sigma_multiplier",
                    3.0,
                )
            ),
        )
        maximum_limit_m = max(
            0.0,
            float(
                rospy.get_param(
                    "/outdoor_waypoint_nav/waypoint_projection_max_distance_m",
                    15.0,
                )
            ),
        )
        sample_sigmas = [legacy.horizontal_stddev(sample) for sample in self.samples]
        horizontal_sigma_m = max(sample_sigmas) if sample_sigmas else 0.0
        adaptive_limit_m = self.adaptive_projection_limit(
            max(0.0, float(self.projection_warn_distance_m)),
            sigma_multiplier,
            maximum_limit_m,
            horizontal_sigma_m,
        )

        try:
            self.tf_listener.waitForTransform(
                self.marker_frame,
                self.robot_frame,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            robot_translation, _ = self.tf_listener.lookupTransform(
                self.marker_frame,
                self.robot_frame,
                rospy.Time(0),
            )
            projection_error = math.hypot(
                marker_point.point.x - robot_translation[0],
                marker_point.point.y - robot_translation[1],
            )
            message = (
                "Waypoint %d GNSS/map check: marker=(%.3f, %.3f), "
                "%s=(%.3f, %.3f), error=%.3f m, sigma=%.3f m, "
                "adaptive_limit=%.3f m"
            )
            values = (
                waypoint_index,
                marker_point.point.x,
                marker_point.point.y,
                self.robot_frame,
                robot_translation[0],
                robot_translation[1],
                projection_error,
                horizontal_sigma_m,
                adaptive_limit_m,
            )
            if projection_error > adaptive_limit_m:
                rospy.logerr(
                    message + "; waypoint rejected because the discrepancy is too large.",
                    *values
                )
                return None

            if projection_error > self.projection_warn_distance_m:
                rospy.logwarn(
                    message
                    + "; accepted by the covariance-aware limit for this GNSS quality.",
                    *values
                )
            else:
                rospy.loginfo(message, *values)
        except (
            legacy.tf.LookupException,
            legacy.tf.ConnectivityException,
            legacy.tf.ExtrapolationException,
        ) as exc:
            rospy.logerr(
                "Waypoint %d rejected: cannot compare GNSS marker with %s in %s: %s",
                waypoint_index,
                self.robot_frame,
                self.marker_frame,
                exc,
            )
            return None

        return marker_point


if __name__ == "__main__":
    legacy.rtk_fix_rejection_reason = total_fix_rejection_reason
    TotalWaypointCollector()
    rospy.spin()
