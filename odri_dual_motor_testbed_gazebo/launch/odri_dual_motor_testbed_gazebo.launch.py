import os
from os import environ, pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Defining the path where the mesh files can be found
    packages = ['odri_dual_motor_testbed_description']
    model_path = get_model_paths(packages)
    gz_model_path_env_var = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', model_path)
    robot_name = 'odri_dual_motor_testbed'

    # Start Gazebo
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch'),
                                       '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -s empty.sdf']}.items()
    )

    gz_sim_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch'),
                                       '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-g']}.items()
    )

    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("odri_dual_motor_testbed_description"),
                    "robots",
                    "odri_dual_motor_testbed.urdf.xacro",
                ]
            ),
            " use_sim:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('odri_dual_motor_testbed_gazebo'),
            'launch'),
                                       '/robot_spawn.launch.py']),
        launch_arguments={'robot_name': robot_name}.items()
    )
    
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller_effort = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["effort_controllers"],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_model_path_env_var,
            gz_sim_server,
            gz_sim_client,
            node_robot_state_publisher,
            robot_spawn,
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

    if 'GZ_SIM_RESOURCE_PATH' in environ:
        model_paths += pathsep + environ['GZ_SIM_RESOURCE_PATH']

    return model_paths
