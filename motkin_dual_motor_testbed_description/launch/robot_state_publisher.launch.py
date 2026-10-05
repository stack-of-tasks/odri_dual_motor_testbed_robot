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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import load_xacro
from launch_ros.actions import Node

# Maps the "robot_model" launch argument to its top-level xacro file under
# robots/. Add an entry here when a new robot model is added.
ROBOT_MODEL_XACRO_FILES = {
    "fivebar_2dof": "fivebar_2dof_robot.urdf.xacro",
    "dual_flywheel": "dual_flywheel_robot.urdf.xacro",
}


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    xacro_file = ROBOT_MODEL_XACRO_FILES[robot_model]

    parameters = {
        "robot_description": load_xacro(
            Path(
                os.path.join(
                    get_package_share_directory(
                        "motkin_dual_motor_testbed_description"
                    ),
                    "robots",
                    xacro_file,
                )
            ),
            mappings={
                "prefix": "",
                "use_sim": "false",
                "use_fake_hardware": "false",
                "fake_sensor_commands": "false",
                "slowdown": "3.0",
            },
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[parameters],
    )

    return [rsp]


def generate_launch_description():
    ld = LaunchDescription()

    declare_robot_model_cmd = DeclareLaunchArgument(
        "robot_model",
        default_value="fivebar_2dof",
        choices=list(ROBOT_MODEL_XACRO_FILES.keys()),
        description="Which robot model to load the description for.",
    )

    ld.add_action(declare_robot_model_cmd)

    # we use OpaqueFunction so the callbacks have access to the context

    # Execute robot_state_publisher node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
