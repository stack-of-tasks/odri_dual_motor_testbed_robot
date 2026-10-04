"""Simulate a single odri_dual_motor_testbed robot in Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("odri_dual_motor_testbed_gazebo"), "launch"
    )
    robot_model = LaunchConfiguration("robot_model")

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            default_value="fivebar_2dof",
            choices=["fivebar_2dof", "dual_flywheel"],
            description="Which robot model to simulate.",
        ),
        DeclareLaunchArgument(
            "spawn_roll",
            default_value="0",
            description="Roll [rad] of the robot when spawned in Gazebo.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the Gazebo GUI client.",
        ),
    ]

    gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "gz_world.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "gui_config": PathJoinSubstitution(
                [
                    FindPackageShare("odri_dual_motor_testbed_gazebo"),
                    "config",
                    [robot_model, ".config"],
                ]
            ),
        }.items(),
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_spawn.launch.py")
        ),
        launch_arguments={
            "robot_model": robot_model,
            "roll": LaunchConfiguration("spawn_roll"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [gz_world, robot_spawn])
