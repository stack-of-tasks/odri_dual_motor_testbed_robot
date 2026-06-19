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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default='fivebar_2dof')
    robot_description = LaunchConfiguration('robot_description', default='')

    # Spawn via -string to avoid a timing race with the robot_description topic:
    # ros_gz_sim create -topic would miss a message already published before the
    # subscriber was set up.  Passing the URDF directly is reliable.
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-model', robot_name,
            '-string', robot_description,
            '-x', '0.1',
            '-y', '0.0',
            '-z', '0.05',
            '-R', '-1.5707963267948966',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='fivebar_2dof'),
        DeclareLaunchArgument('robot_description', default_value=''),
        gazebo_spawn_robot,
    ])
