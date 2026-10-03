# Copyright 2026
# Licensed under the Apache License, Version 2.0.
"""Run the two-phase execution demo: an in-memory robot plus a three-level cascade.

    ros2 launch controller_manager two_phase_demo.launch.py

The controllers are spawned leaf-first, because a chainable controller's reference interfaces must
exist before the parent that claims them can be configured.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("controller_manager")
    demo_dir = os.path.join(share, "two_phase_demo")
    urdf_path = os.path.join(demo_dir, "two_phase_demo.urdf")
    yaml_path = os.path.join(demo_dir, "two_phase_demo_controllers.yaml")

    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description, "update_rate": 100.0}, yaml_path],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", yaml_path, "--controller-manager-timeout", "30"],
            output="screen",
        )

    # Chain the spawners so the child is configured before its parent claims its interfaces.
    leaf = spawner("tp_leaf")
    mid = spawner("tp_mid")
    root = spawner("tp_root")

    return LaunchDescription(
        [
            control_node,
            leaf,
            RegisterEventHandler(OnProcessExit(target_action=leaf, on_exit=[mid])),
            RegisterEventHandler(OnProcessExit(target_action=mid, on_exit=[root])),
        ]
    )
