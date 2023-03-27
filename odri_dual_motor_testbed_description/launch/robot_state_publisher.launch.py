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

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from launch_param_builder import load_xacro
from launch_ros.actions import Node




def launch_setup(context, *args, **kwargs):

    parameters = {'robot_description': load_xacro(
        Path(os.path.join(
            get_package_share_directory('odri_dual_motor_testbed_description'),
            'robots',
            'odri_dual_motor_testbed.urdf.xacro')),
    )}

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[parameters])

    return [rsp]


def generate_launch_description():

    ld = LaunchDescription()

    # we use OpaqueFunction so the callbacks have access to the context

    # Execute robot_state_publisher node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
