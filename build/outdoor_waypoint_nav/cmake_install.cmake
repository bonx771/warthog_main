# Install script for directory: /home/cuong/warthog/src/outdoor_waypoint_nav

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cuong/warthog/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/outdoor_waypoint_nav.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/outdoor_waypoint_nav/cmake" TYPE FILE FILES
    "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/outdoor_waypoint_navConfig.cmake"
    "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/outdoor_waypoint_navConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/outdoor_waypoint_nav" TYPE FILE FILES "/home/cuong/warthog/src/outdoor_waypoint_nav/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/outdoor_waypoint_nav" TYPE DIRECTORY FILES "/home/cuong/warthog/src/outdoor_waypoint_nav/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/outdoor_waypoint_nav" TYPE DIRECTORY FILES "/home/cuong/warthog/src/outdoor_waypoint_nav/params")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/outdoor_waypoint_nav" TYPE DIRECTORY FILES "/home/cuong/warthog/src/outdoor_waypoint_nav/worlds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/camera_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/position_error.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/encoder_odometry_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/experiment_trajectory_csv_logger.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/filtered_collect_gps_waypoints.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/gnss_correlated_noise.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/gnss_outage_gate.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/gnss_periodic_gate.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/gps_kalman_filter.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/gps_covariance_relay.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/home_to_initial_waypoint.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/imu_covariance_log.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/imu_covariance_relay.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/inovation.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/inovation_sim.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/joy_launch_control.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/joy_launch_control_continuous.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/joy_launch_control_mapping.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/keyboard_to_joy.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/path_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/plot_results.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/square_eval.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/stationary_gps_gate.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/tracking_control.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/return_home.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/record_gps_waypoints.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/outdoor_waypoint_nav" TYPE PROGRAM FILES "/home/cuong/warthog/build/outdoor_waypoint_nav/catkin_generated/installspace/scenario_4_waypoint_manager.py")
endif()

