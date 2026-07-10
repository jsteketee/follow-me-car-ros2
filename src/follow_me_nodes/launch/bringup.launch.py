#!/usr/bin/env python3
"""bringup.launch.py — start the full car stack in one command.

Brings up:
  robot_state_publisher  URDF -> /tf_static (base_link -> imu_link, wheels, uwb_link)
                         and -> /robot_description, which Foxglove renders as the car
  serial_bridge          ESP32 JSON telemetry -> ROS2 topics
  pose_estimator         IMU heading + wheel odometry -> odom -> base_link on /tf
  foxglove_bridge        websocket for the Foxglove 3D panel (optional)

Launch arguments:
  namespace    robot namespace for the nodes' topics (default: none)
  foxglove     start foxglove_bridge alongside (default: true)
  serial_port  override the ESP32 serial device (default: serial_bridge's own default)

Usage:
  ros2 launch follow_me_nodes bringup.launch.py
  ros2 launch follow_me_nodes bringup.launch.py namespace:=fmbot foxglove:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the full car stack."""
    pkg_share = get_package_share_directory("follow_me_nodes")
    urdf_path = os.path.join(pkg_share, "urdf", "follow_me_car.urdf")

    # robot_state_publisher wants the URDF's XML *contents* as a string parameter,
    # not a path — hand it a path and it silently fails to parse, publishing no
    # /tf_static and leaving imu_link dangling again. So read the file here.
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    namespace = LaunchConfiguration("namespace")
    foxglove = LaunchConfiguration("foxglove")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="Robot namespace applied to node topics (frame ids are unaffected).",
        ),
        DeclareLaunchArgument(
            "foxglove", default_value="true",
            description="Also start foxglove_bridge.",
        ),

        # Publishes the URDF's fixed joints onto /tf_static, and the URDF itself onto
        # /robot_description (latched). Deliberately does NOT publish odom -> base_link:
        # that edge belongs to pose_estimator, and two publishers on one edge fight.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="follow_me_nodes",
            executable="serial_bridge",
            name="serial_bridge",
            namespace=namespace,
            output="screen",
        ),

        Node(
            package="follow_me_nodes",
            executable="pose_estimator",
            name="pose_estimator",
            namespace=namespace,
            output="screen",
        ),

        # Not namespaced: the bridge serves the whole graph regardless of robot.
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            condition=IfCondition(foxglove),
        ),
    ])
