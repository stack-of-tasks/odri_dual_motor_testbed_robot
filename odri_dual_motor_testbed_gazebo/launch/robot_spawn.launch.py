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
"""Spawn one robot, under an optional ROS namespace, in a running Gazebo world.

Everything that belongs to a single robot instance is started here:
robot_state_publisher, the Gazebo entity, and the controller spawners.
Including this file several times with different ``namespace`` values puts
several independent robots in the same world (see
odri_dual_motor_testbed_haptic_pair).

The namespace is forwarded to the xacro (``robot_namespace``), which puts it
in the odri_gz_ros2_control plugin's ``<ros><namespace>``: the robot's
controller_manager then lives at ``/<namespace>/controller_manager``.  The
controller params YAML must therefore key its entries under ``/**:`` so they
still match once the nodes are namespaced.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    controllers = LaunchConfiguration("controllers").perform(context).split()
    controller_params = LaunchConfiguration("controller_params_file")

    # Gazebo entity names must be unique in the world.
    if not robot_name:
        robot_name = f"{namespace}_{robot_model}" if namespace else robot_model
    controller_manager = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("odri_dual_motor_testbed_description"), "robots"]
            ),
            "/",
            robot_model,
            "_robot.urdf.xacro",
            " gz_sim:=true",
            " robot_namespace:=",
            namespace,
            " controller_params_file:=",
            controller_params,
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                )
            },
            {"use_sim_time": True},
        ],
    )

    # Spawn via -string to avoid a timing race with the robot_description topic:
    # ros_gz_sim create -topic would miss a message already published before the
    # subscriber was set up.  Passing the URDF directly is reliable.
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-string",
            robot_description_content,
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("roll"),
            "-P",
            LaunchConfiguration("pitch"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=[
                controller,
                "--controller-manager",
                controller_manager,
                "--param-file",
                controller_params,
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
        for controller in ["joint_state_broadcaster", *controllers]
    ]

    return [node_robot_state_publisher, gazebo_spawn_robot, *spawners]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace of this robot instance (empty: root namespace).",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="fivebar_2dof",
            choices=["fivebar_2dof", "dual_flywheel"],
            description="Which robot model to spawn.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="",
            description="Name of the model in Gazebo "
            "(empty: <namespace>_<robot_model>, or <robot_model> without namespace).",
        ),
        DeclareLaunchArgument(
            "controller_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("odri_dual_motor_testbed_gazebo"),
                    "config",
                    "forward_command_controller.yaml",
                ]
            ),
            description="controller_manager YAML loaded by odri_gz_ros2_control.",
        ),
        DeclareLaunchArgument(
            "controllers",
            default_value="odri_forward_command_controller",
            description="Space-separated controllers to spawn after joint_state_broadcaster.",
        ),
        DeclareLaunchArgument("x", default_value="0.1"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.00"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
