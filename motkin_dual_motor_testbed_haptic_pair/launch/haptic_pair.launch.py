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
"""
Run two real motkin_dual_motor_testbed five-bar kits as a haptic pair.

Real-hardware counterpart of haptic_pair_gazebo.launch.py: both kits are
plugged into the *same* computer, and each one gets its own
ros2_control_node, robot_state_publisher and controller spawners under the
"leader" and "follower" namespaces, so that their controller_manager,
joint_states, tf and controller topics never collide.

Since two motkin boards are connected, the serial port auto-detection of
the hardware interface would pick the same board twice: the serial device
of each kit must be given explicitly (leader_serial_port and
follower_serial_port). Prefer the stable /dev/serial/by-id/usb-... paths
over /dev/ttyACM*, whose numbering depends on the plug order.

Both instances load the same motkin_five_bar_force_velocity_controller
(config/haptic_pair_controllers.yaml). Publish a
geometry_msgs/WrenchStamped on
"/leader/motkin_five_bar_force_velocity_controller/contact_force" to move
the leader; contact_force_relay republishes it onto the follower's own
contact_force topic so that the follower reproduces the same motion.

With rviz:=true, one RViz window is opened per kit, each one showing the
robot of its own namespace.
"""

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Structural constants: everything downstream (the contact_force_relay
# topics) is wired to these two specific roles.
LEADER_NAMESPACE = "leader"
FOLLOWER_NAMESPACE = "follower"
CONTROLLER = "motkin_five_bar_force_velocity_controller"


def robot_nodes(namespace, serial_port, controller_params, rviz):
    """Return the nodes driving one real kit under ``namespace``."""
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("motkin_dual_motor_testbed_description"),
                            "robots",
                            "fivebar_2dof_robot.urdf.xacro",
                        ]
                    ),
                    " serial_port:=",
                    serial_port,
                ]
            ),
            value_type=str,
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, controller_params],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[robot_description],
        # tf2 publishes on the absolute /tf: keep each robot's tree in its
        # namespace.
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=[
                controller,
                "--controller-manager",
                f"/{namespace}/controller_manager",
            ],
            output="screen",
        )
        for controller in ["joint_state_broadcaster", CONTROLLER]
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("motkin_dual_motor_testbed_description"),
                    "rviz",
                    "display_motkin_dual_motor_testbed.rviz",
                ]
            ),
        ],
        # The RViz configuration uses absolute topic names.
        remappings=[
            ("/robot_description", "robot_description"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        condition=IfCondition(rviz),
    )

    return [control_node, robot_state_publisher, *spawners, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "leader_serial_port",
            description="Serial device of the leader's motkin board, "
            "e.g. /dev/serial/by-id/usb-... (required: no auto-detection "
            "with two boards).",
        ),
        DeclareLaunchArgument(
            "follower_serial_port",
            description="Serial device of the follower's motkin board, "
            "e.g. /dev/serial/by-id/usb-... (required: no auto-detection "
            "with two boards).",
        ),
        DeclareLaunchArgument(
            "controller_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("motkin_dual_motor_testbed_haptic_pair"),
                    "config",
                    "haptic_pair_controllers.yaml",
                ]
            ),
            description="controller_manager YAML loaded by both robot instances.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open one RViz window per kit.",
        ),
    ]

    controller_params = LaunchConfiguration("controller_params_file")
    rviz = LaunchConfiguration("rviz")

    contact_force_relay = Node(
        package="motkin_dual_motor_testbed_haptic_pair",
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
        + robot_nodes(
            LEADER_NAMESPACE,
            LaunchConfiguration("leader_serial_port"),
            controller_params,
            rviz,
        )
        + robot_nodes(
            FOLLOWER_NAMESPACE,
            LaunchConfiguration("follower_serial_port"),
            controller_params,
            rviz,
        )
        + [contact_force_relay]
    )
