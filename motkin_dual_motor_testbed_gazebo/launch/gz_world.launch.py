# Copyright (c) 2026 CNRS All rights reserved.
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
"""Start an empty Gazebo world ready to receive robots from robot_spawn.launch.py.

Sets the Gazebo resource/plugin paths, starts the server (and optionally the
GUI) and bridges /clock.  Robots are added separately, one include of
robot_spawn.launch.py per robot.
"""

import os
from os import environ, pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the Gazebo GUI client.",
        ),
        DeclareLaunchArgument(
            "gui_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("motkin_dual_motor_testbed_gazebo"),
                    "config",
                    "gui.config",
                ]
            ),
            description="Gazebo GUI configuration file.",
        ),
    ]

    # Defining the path where the mesh files can be found
    gz_model_path_env_var = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", get_model_paths(["motkin_dual_motor_testbed_description"])
    )

    # FiveBarClosurePlugin (this package) and the motkin_gz_ros2_control system
    # plugin live in different install prefixes unless --merge-install is used.
    gz_sim_sys_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        get_plugin_paths(["motkin_dual_motor_testbed_gazebo", "motkin_gz_ros2_control"]),
    )

    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r -s empty.sdf"]}.items(),
    )

    gz_sim_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args": ["-g --gui-config ", LaunchConfiguration("gui_config")]
        }.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gz_model_path_env_var,
            gz_sim_sys_plugin_path,
            gz_sim_server,
            gz_sim_client,
            clock_bridge,
        ]
    )


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    if "GZ_SIM_RESOURCE_PATH" in environ:
        model_paths += pathsep + environ["GZ_SIM_RESOURCE_PATH"]

    return model_paths


def get_plugin_paths(packages_names):
    plugin_paths = [
        os.path.join(get_package_prefix(package_name), "lib")
        for package_name in packages_names
    ]
    if "GZ_SIM_SYSTEM_PLUGIN_PATH" in environ:
        plugin_paths.append(environ["GZ_SIM_SYSTEM_PLUGIN_PATH"])
    return pathsep.join(dict.fromkeys(plugin_paths))
