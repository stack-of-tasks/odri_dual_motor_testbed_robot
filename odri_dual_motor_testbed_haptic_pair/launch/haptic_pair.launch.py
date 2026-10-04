# Copyright 2026 LAAS-CNRS
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
"""Simulate two odri_dual_motor_testbed five-bar robots as a haptic pair.

One Gazebo world is started with odri_dual_motor_testbed_gazebo's
gz_world.launch.py, then odri_dual_motor_testbed_gazebo's
robot_spawn.launch.py is included twice, once per robot, under the
"leader" and "follower" namespaces so that their controller_manager,
joint_states, tf and controller topics never collide. Each instance loads
the *same*, existing odri_five_bar_force_velocity_controller (see
config/haptic_pair_controllers.yaml), which already implements the
force-to-motion admittance law (qdot = J^T f_c) that makes a five-bar act
as a haptic device.

The leader is the one meant to receive external forces -- publish a
geometry_msgs/WrenchStamped on
"/leader/odri_five_bar_force_velocity_controller/contact_force" and the
leader moves accordingly, acting as a haptic sensor.

contact_force_relay republishes the leader's sensed contact force onto the
follower's own contact_force topic. Since the follower runs the identical
controller, it reproduces the same force -- and, by construction of the
shared admittance law, the same motion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Structural constants: everything downstream (the contact_force_relay
# topics) is wired to these two specific roles.
LEADER_NAMESPACE = "leader"
FOLLOWER_NAMESPACE = "follower"
CONTROLLER = "odri_five_bar_force_velocity_controller"


def generate_launch_description():
    gazebo_launch_dir = os.path.join(
        get_package_share_directory("odri_dual_motor_testbed_gazebo"), "launch"
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "controller_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("odri_dual_motor_testbed_haptic_pair"),
                    "config",
                    "haptic_pair_controllers.yaml",
                ]
            ),
            description="controller_manager YAML loaded by both robot instances.",
        ),
        DeclareLaunchArgument(
            "leader_x",
            default_value="-0.15",
            description="X position (m) of the leader robot.",
        ),
        DeclareLaunchArgument(
            "follower_x",
            default_value="0.15",
            description="X position (m) of the follower robot.",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.01",
            description="Z position (m) of both robots.",
        ),
        DeclareLaunchArgument(
            "spawn_roll", default_value="0", description="Roll [rad] of both robots."
        ),
        DeclareLaunchArgument(
            "gui", default_value="true", description="Start the Gazebo GUI client."
        ),
        DeclareLaunchArgument(
            "gui_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("odri_dual_motor_testbed_haptic_pair"),
                    "config",
                    "haptic_pair.config",
                ]
            ),
            description="Gazebo GUI configuration file.",
        ),
    ]

    gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, "gz_world.launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "gui_config": LaunchConfiguration("gui_config"),
        }.items(),
    )

    def robot_spawn(namespace, x):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_launch_dir, "robot_spawn.launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "robot_model": "fivebar_2dof",
                "controller_params_file": LaunchConfiguration("controller_params_file"),
                "controllers": CONTROLLER,
                "x": x,
                "y": "0.0",
                "z": LaunchConfiguration("spawn_z"),
                "roll": LaunchConfiguration("spawn_roll"),
            }.items(),
        )

    contact_force_relay = Node(
        package="odri_dual_motor_testbed_haptic_pair",
        executable="contact_force_relay",
        name="contact_force_relay",
        output="screen",
        parameters=[
            {
                "input_topic": f"/{LEADER_NAMESPACE}/{CONTROLLER}/contact_force",
                "output_topic": f"/{FOLLOWER_NAMESPACE}/{CONTROLLER}/contact_force",
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gz_world,
            robot_spawn(LEADER_NAMESPACE, LaunchConfiguration("leader_x")),
            robot_spawn(FOLLOWER_NAMESPACE, LaunchConfiguration("follower_x")),
            contact_force_relay,
        ]
    )
