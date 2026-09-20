# Copyright (c) 2023 LAAS/CNRS All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "odri_dual_motor_testbed_description"
    pkg_dir = FindPackageShare(pkg_name)
    paths = ["launch", "robot_state_publisher.launch.py"]
    full_path = PathJoinSubstitution([pkg_dir] + paths)
    robot_state_publisher = IncludeLaunchDescription(
        full_path,
        launch_arguments={
            "robot_model": LaunchConfiguration("robot_model"),
        }.items(),
    )

    declare_robot_model_cmd = DeclareLaunchArgument(
        "robot_model",
        default_value="fivebar_2dof",
        choices=["fivebar_2dof", "dual_flywheel"],
        description="Which robot model to load the description for.",
    )

    default_rviz_config_file = PathJoinSubstitution(
        [pkg_dir, "rviz", "display_odri_dual_motor_testbed.rviz"]
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_config_file,
        description="Full path to the RViz config file to use",
    )

    start_joint_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(start_joint_pub_gui)
    ld.add_action(start_rviz_cmd)

    return ld
