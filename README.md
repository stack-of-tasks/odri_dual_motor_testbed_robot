# ODRI Dual motor testbed robot's ros2-control package 

This repository aims at providing a ros2-control package for the ODRI dual motor testbed robot.

It allows to have bolt displayed through rviz, provides the access to the ros2_controllers. The most notorious is joint_state_broadcaster which provides the topic /joint_states for free. It is then possible the node robot_state_publisher to have the TF-2 tree of the Bolt robot and to display it on rviz.

## Dependencies

This repositor relies on [ros2_control](https://github.com/ros-controls/ros2_control) and [ros2_controllers](https://github.com/ros-controls/ros2_controllers) regarding the ros2-control framework.
It relies more specifically on the [ros2_hardware_interface_odri](https://github.com/stack-of-tasks/ros2_hardware_interface_odri) to handle the relationship with the [master-board](https://github.com/open-dynamic-robot-initiative/master-board)
developped by the Open Dynamic Robot Initiative.



