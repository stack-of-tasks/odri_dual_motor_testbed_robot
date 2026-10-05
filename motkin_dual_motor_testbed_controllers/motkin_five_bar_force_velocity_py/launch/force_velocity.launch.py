from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    forward_command_controller_name_arg = DeclareLaunchArgument(
        "forward_command_controller_name",
        default_value="motkin_forward_command_controller",
        description="Name of the running motkin_forward_command_controller "
        "instance to publish velocity commands to.",
    )
    contact_force_topic_arg = DeclareLaunchArgument(
        "contact_force_topic",
        default_value="~/contact_force",
        description="geometry_msgs/WrenchStamped topic providing the "
        "contact force f_c = (fx, fy) at the closing point, "
        "expressed in the (x, y) plane of case.",
    )
    gain_kd_arg = DeclareLaunchArgument(
        "gain_kd",
        default_value="0.05",
        description="Velocity gain sent alongside the qdot command.",
    )

    node = Node(
        package="motkin_five_bar_force_velocity_py",
        executable="force_velocity_node",
        name="motkin_five_bar_force_velocity_node",
        output="screen",
        parameters=[
            {
                "forward_command_controller_name": LaunchConfiguration(
                    "forward_command_controller_name"
                ),
                "contact_force_topic": LaunchConfiguration("contact_force_topic"),
                "gain_kd": LaunchConfiguration("gain_kd"),
            }
        ],
    )

    return LaunchDescription(
        [
            forward_command_controller_name_arg,
            contact_force_topic_arg,
            gain_kd_arg,
            node,
        ]
    )
