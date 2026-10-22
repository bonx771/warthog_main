#!/usr/bin/env python3
"""Relay every valid GNSS solution and expose its current quality state.

RTK FLOAT and RTK FIXED share ``STATUS_GBAS_FIX`` in ``NavSatFix``.  The GPS
driver gives them different covariance floors, so this relay only uses the
reported horizontal standard deviation to label them for the operator.  The
message itself is never modified: localization continues to use the receiver's
real covariance.

NO_FIX and malformed fixes are not relayed.  This is deliberate: during an
outage robot_localization keeps propagating the map filter from wheel odometry
and IMU instead of consuming an invalid or repeated GNSS position.
"""

import math

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String


class GnssStateRelayTotal(object):
    LOST = "GPS_LOST"
    NORMAL = "GPS_NORMAL"
    FLOAT = "RTK_FLOAT"
    FIXED = "RTK_FIXED"

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/gps/fix")
        self.output_topic = rospy.get_param(
            "~output_topic", "/outdoor_waypoint_nav/gps/fix_selected"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/outdoor_waypoint_nav/gps/state_total"
        )
        self.available_topic = rospy.get_param(
            "~available_topic", "/outdoor_waypoint_nav/gps/available_total"
        )
        self.outage_active_topic = rospy.get_param(
            "~outage_active_topic", "/outdoor_waypoint_nav/gps/outage_active_total"
        )
        self.message_timeout = max(
            0.0, float(rospy.get_param("~message_timeout", 1.5))
        )
        self.fixed_max_horizontal_stddev = max(
            0.0,
            float(rospy.get_param("~fixed_max_horizontal_stddev", 0.20)),
        )

        self.last_message_time = None
        self.current_state = None
        self.fix_publisher = rospy.Publisher(
            self.output_topic, NavSatFix, queue_size=20
        )
        self.state_publisher = rospy.Publisher(
            self.state_topic, String, queue_size=1, latch=True
        )
        self.available_publisher = rospy.Publisher(
            self.available_topic, Bool, queue_size=1, latch=True
        )
        self.outage_active_publisher = rospy.Publisher(
            self.outage_active_topic, Bool, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            NavSatFix,
            self.fix_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timeout_callback)

        self.publish_state(self.LOST, "waiting for the first valid GNSS fix")
        rospy.loginfo(
            "GNSS total relay: %s -> %s; covariance is preserved; state topic=%s",
            self.input_topic,
            self.output_topic,
            self.state_topic,
        )

    @staticmethod
    def valid_position(msg):
        return (
            msg.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        )

    @staticmethod
    def horizontal_stddev(msg):
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None
        covariance_x = float(msg.position_covariance[0])
        covariance_y = float(msg.position_covariance[4])
        if (
            not math.isfinite(covariance_x)
            or not math.isfinite(covariance_y)
            or covariance_x < 0.0
            or covariance_y < 0.0
        ):
            return None
        return math.sqrt(max(covariance_x, covariance_y))

    def classify(self, msg):
        if not self.valid_position(msg):
            return self.LOST

        if msg.status.status >= NavSatStatus.STATUS_GBAS_FIX:
            horizontal_stddev = self.horizontal_stddev(msg)
            if (
                horizontal_stddev is not None
                and horizontal_stddev <= self.fixed_max_horizontal_stddev
            ):
                return self.FIXED
            return self.FLOAT

        return self.NORMAL

    def publish_state(self, state, reason=""):
        if state == self.current_state:
            return
        previous_state = self.current_state or "STARTUP"
        self.current_state = state
        self.state_publisher.publish(String(data=state))
        self.available_publisher.publish(Bool(data=state != self.LOST))
        self.outage_active_publisher.publish(Bool(data=state == self.LOST))
        if reason:
            rospy.logwarn(
                "GNSS state: %s -> %s (%s)", previous_state, state, reason
            )
        else:
            rospy.loginfo("GNSS state: %s -> %s", previous_state, state)

    def fix_callback(self, msg):
        self.last_message_time = rospy.Time.now()
        state = self.classify(msg)
        if state == self.LOST:
            self.publish_state(self.LOST, "receiver reports NO_FIX or invalid position")
            return

        self.publish_state(state)
        self.fix_publisher.publish(msg)

    def timeout_callback(self, _event):
        if self.message_timeout <= 0.0 or self.last_message_time is None:
            return
        age = (rospy.Time.now() - self.last_message_time).to_sec()
        if age > self.message_timeout:
            self.publish_state(
                self.LOST,
                "no GNSS message for {:.1f} s".format(age),
            )


if __name__ == "__main__":
    rospy.init_node("gnss_state_relay_total")
    GnssStateRelayTotal()
    rospy.spin()
