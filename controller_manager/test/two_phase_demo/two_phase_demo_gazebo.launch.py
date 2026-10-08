# Copyright 2026
# Licensed under the Apache License, Version 2.0.
"""Run the two-phase cascade on REAL Gazebo physics.

    ros2 launch controller_manager two_phase_demo_gazebo.launch.py

Same three-level cascade as `two_phase_demo.launch.py`, with two differences that make it the
deployment case rather than an in-process one:

  * the hardware is `gazebo_ros2_control/GazeboSystem` inside `gzserver`, so `joint2` is moved by
    Gazebo physics and its state comes back through real hardware interfaces, and
  * the ControllerManager lives inside `gzserver` (`gazebo_ros2_control` creates it) and reads
    `two_phase_demo_gazebo_controllers.yaml` through the plugin's `<parameters>` tag, so the
    controllers are created by the spawner over DDS, from the plugin, by type string.

The cascade is driven from outside by publishing on the chain ROOT's reference topic:

    ros2 topic pub --once /tp_root/reference std_msgs/msg/Float64 "{data: 0.75}"

which only has an effect because `refresh_reference_phase()` exists in the two-phase contract.
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("controller_manager")
    demo_dir = os.path.join(share, "two_phase_demo")
    urdf_path = os.path.join(demo_dir, "two_phase_demo_gazebo.urdf")
    yaml_path = os.path.join(demo_dir, "two_phase_demo_gazebo_controllers.yaml")

    with open(urdf_path, "r", encoding="utf-8") as handle:
        # The plugin reads the file itself, so it needs a real path; plain URDF does not expand
        # `$(find ...)`, and spawning through xacro would add a dependency for one string.
        # `gazebo_ros2_control` hands the description to its own ControllerManager as a command-line
        # parameter override, and rcl parses that value as YAML. Three things therefore have to go
        # before it is published, all of them cosmetic for URDF parsing but fatal for the parser:
        #   * the XML declaration -- a leading `?` makes YAML read a complex mapping key;
        #   * comments -- enough of them, and the override is truncated by rclcutils;
        #   * newlines -- the override is a single command-line token.
        # The file on disk keeps its comments for the reader; only the published string is stripped.
        text = handle.read().replace("$CONTROLLERS_YAML", yaml_path)
        text = re.sub(r"<\?xml[^>]*\?>", "", text)
        text = re.sub(r"<!--.*?-->", "", text, flags=re.DOTALL)
        robot_description = " ".join(text.split())

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Headless: no GUI, and `-s` loads the ROS init (publishes /clock) and the entity factory.
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "two_phase_gazebo",
            "-timeout",
            "120",
        ],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--param-file",
                yaml_path,
                "--controller-manager-timeout",
                "120",
            ],
            output="screen",
        )

    # `gazebo_ros2_control` starts its ControllerManager when the model is inserted, so the spawners
    # wait for the SPAWN to finish rather than just for gzserver.
    leaf = spawner("tp_leaf")
    mid = spawner("tp_mid")
    root = spawner("tp_root")

    return LaunchDescription(
        [
            gzserver,
            robot_state_publisher,
            spawn_entity,
            RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[leaf])),
            RegisterEventHandler(OnProcessExit(target_action=leaf, on_exit=[mid])),
            RegisterEventHandler(OnProcessExit(target_action=mid, on_exit=[root])),
        ]
    )
