#!/usr/bin/env python3
"""bringup.launch.py — launch the full car stack (state publisher, serial bridge,
estimators, optional foxglove). Args: namespace, foxglove, serial_port."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Import DEFAULT_PORT from the bridge node so launch and node defaults stay in sync.
from follow_me_nodes.serial_bridge import DEFAULT_PORT


def generate_launch_description():
    """Build the launch description for the full car stack."""
    pkg_share = get_package_share_directory("follow_me_nodes")
    urdf_path = os.path.join(pkg_share, "urdf", "follow_me_car.urdf")

    # robot_state_publisher needs the URDF contents (not a path), so read the file here.
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    namespace = LaunchConfiguration("namespace")
    foxglove = LaunchConfiguration("foxglove")
    follow = LaunchConfiguration("follow")
    serial_port = LaunchConfiguration("serial_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="fmbot",
            description="Robot namespace applied to node topics (frame ids are unaffected).",
        ),
        DeclareLaunchArgument(
            "foxglove", default_value="true",
            description="Also start foxglove_bridge.",
        ),
        DeclareLaunchArgument(
            "follow", default_value="false",
            description="Also start nav_controller — it publishes cmd_drive and DRIVES the car.",
        ),
        DeclareLaunchArgument(
            "serial_port", default_value=DEFAULT_PORT,
            description="ESP32 serial device path (default: serial_bridge's own default).",
        ),

        # Publishes URDF fixed joints to /tf_static and the URDF to /robot_description; odom->base_link is pose_estimator's.
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
            parameters=[{"serial_port": serial_port}],
        ),

        Node(
            package="follow_me_nodes",
            executable="pose_estimator",
            name="pose_estimator",
            namespace=namespace,
            output="screen",
        ),

        Node(
            package="follow_me_nodes",
            executable="tag_broadcaster",
            name="tag_broadcaster",
            namespace=namespace,
            output="screen",
        ),

        Node(
            package="follow_me_nodes",
            executable="tag_estimator",
            name="tag_estimator",
            namespace=namespace,
            output="screen",
        ),

        # Off by default (follow:=true): publishes cmd_drive and will drive the car.
        Node(
            package="follow_me_nodes",
            executable="nav_controller",
            name="nav_controller",
            namespace=namespace,
            output="screen",
            condition=IfCondition(follow),
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
