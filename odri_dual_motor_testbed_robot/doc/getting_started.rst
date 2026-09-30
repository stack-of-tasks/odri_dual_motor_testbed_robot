Getting started
===============

Installation
------------

In a ROS 2 Jazzy workspace:

.. code-block:: bash

   mkdir -p odri_dual_motor_testbed_ws/src
   cd odri_dual_motor_testbed_ws/src
   git clone https://github.com/stack-of-tasks/odri_dual_motor_testbed_robot.git
   cd ..
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source ./install/setup.bash

Two ``.repos`` files, at the root of the repository, fetch the other
repositories into the workspace. Run ``vcs import`` from the workspace root,
before ``rosdep`` and ``colcon build``:

.. code-block:: bash

   cd odri_dual_motor_testbed_ws
   # Simulation: odri_gz_ros2_control
   vcs import < src/odri_dual_motor_testbed_robot/odri_dual_motor_testbed_gazebo.repos
   # Real robot: pico_dual_drv8316c_ros2_hardware_interface and the ODRI stack
   vcs import < src/odri_dual_motor_testbed_robot/odri_dual_motor_testbed_robot.repos

Display the robot
-----------------

.. code-block:: bash

   ros2 launch odri_dual_motor_testbed_description show.launch.py robot_model:=fivebar_2dof

See the ``odri_dual_motor_testbed_description`` documentation.

Simulate the robot
------------------

.. code-block:: bash

   ros2 launch odri_dual_motor_testbed_gazebo odri_dual_motor_testbed_gazebo.launch.py robot_model:=fivebar_2dof

See the ``odri_dual_motor_testbed_gazebo`` documentation.

Start the real robot
--------------------

.. code-block:: bash

   ros2 launch odri_dual_motor_testbed_bringup odri_dual_motor_testbed_ctrl.launch.py robot_model:=fivebar_2dof

See the ``odri_dual_motor_testbed_bringup`` documentation.

Send a command
--------------

In simulation or on the real robot, hold both motors at 0 with a PD:

.. code-block:: bash

   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  5.0, 5.0,  0.1, 0.1]}"

See the ``odri_forward_command_controller`` documentation for the message
layout.
