Getting started
===============

Installation
------------

In a ROS 2 Jazzy workspace:

.. code-block:: bash

   mkdir -p motkin_dual_motor_testbed_ws/src
   cd motkin_dual_motor_testbed_ws/src
   git clone https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot.git
   cd ..
   vcs import --skip-existing src < src/motkin_dual_motor_testbed_robot/motkin_dual_motor_testbed_robot.repos
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source ./install/setup.bash

``motkin_dual_motor_testbed_robot.repos``, at the root of the repository, lists
the other repositories the workspace needs:

* ``motkin_gz_ros2_control``: ``ros2_control`` hardware interface for Gazebo,
  needed for simulation;
* ``motkin_ros2_hardware_interface``: hardware interface of the
  motor board, needed for the real robot.

The file also lists this repository, so ``--skip-existing`` keeps the clone
made above untouched.

Display the robot
-----------------

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_description show.launch.py robot_model:=fivebar_2dof

See the ``motkin_dual_motor_testbed_description`` documentation.

Simulate the robot
------------------

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_gazebo motkin_dual_motor_testbed_gazebo.launch.py robot_model:=fivebar_2dof

See the ``motkin_dual_motor_testbed_gazebo`` documentation.

Start the real robot
--------------------

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_bringup motkin_dual_motor_testbed_ctrl.launch.py robot_model:=fivebar_2dof

See the ``motkin_dual_motor_testbed_bringup`` documentation.

Send a command
--------------

In simulation or on the real robot, hold both motors at 0 with a PD:

.. code-block:: bash

   ros2 topic pub --once /motkin_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  5.0, 5.0,  0.1, 0.1]}"

See the ``motkin_forward_command_controller`` documentation for the message
layout.
