#!/usr/bin/env python3
"""Obstacle controller entry point for the all-GNSS total scenario."""

import os

import joy_launch_control as controller
import joy_launch_control_obstacle as obstacle_controller
import rospkg
import rospy


def main():
    controller.getParameter()
    controller.getPaths()
    obstacle_controller.install_obstacle_navigation_gate()

    package_path = rospkg.RosPack().get_path("outdoor_waypoint_nav")
    controller.location_collect = os.path.join(
        package_path, "launch/include/collect_goals_total.launch"
    )
    controller.location_send = os.path.join(
        package_path, "launch/include/send_goals_total.launch"
    )

    controller.print_instructions()
    controller.main()


if __name__ == "__main__":
    main()
