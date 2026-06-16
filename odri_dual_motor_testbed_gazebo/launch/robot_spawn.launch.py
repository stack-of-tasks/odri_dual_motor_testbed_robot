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

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from dataclasses import dataclass
from launch.logging import get_logger

def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    declare_actions(ld)

    return ld


def declare_actions(
    launch_description: LaunchDescription
):

    logger = get_logger('robot_spawn - gazebo')

    arguments=[
            "-model odri_dual_motor_testbed",
            "-topic",
            "robot_description",
            "-x","0.1",
            "-y","0.0",
            "-z","0.1", 
        ]
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=arguments,
    )

    logger.info("arguments:" + str(arguments))

    launch_description.add_action(gazebo_spawn_robot)
    return
