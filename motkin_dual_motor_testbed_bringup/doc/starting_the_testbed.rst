Starting the testbed
====================

Before launching
----------------

#. Power the motor board and plug its USB cable into the computer.
#. Mount the mechanism that matches the model you are going to load: the
   five-bar linkage (``fivebar_2dof``) or the two flywheels
   (``dual_flywheel``).
#. Source the workspace:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source motkin_dual_motor_testbed_ws/install/setup.bash

Launching
---------

To start the robot with the command controller:

.. code-block:: bash

   # Five-bar linkage (default)
   ros2 launch motkin_dual_motor_testbed_bringup motkin_dual_motor_testbed_ctrl.launch.py

   # Dual flywheel
   ros2 launch motkin_dual_motor_testbed_bringup motkin_dual_motor_testbed_ctrl.launch.py robot_model:=dual_flywheel

This starts, in order:

#. ``ros2_control_node`` (the ``controller_manager``), which loads the robot
   description and connects to the motor board;
#. the ``joint_state_broadcaster`` spawner;
#. ``robot_state_publisher``;
#. once ``joint_state_broadcaster`` is active, the
   ``motkin_forward_command_controller`` spawner;
#. ``rviz2``, with ``rviz/display_motkin_dual_motor_testbed.rviz`` from the
   description package.

To only read the joint states, without a command controller, use
``motkin_dual_motor_testbed.launch.py`` instead. It starts the same nodes except
``motkin_forward_command_controller``, and opens RViz once
``joint_state_broadcaster`` is active.

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_bringup motkin_dual_motor_testbed.launch.py robot_model:=fivebar_2dof

Several testbeds on one computer
--------------------------------

Start one launch file per testbed, each with its own ``namespace`` and the
``serial_port`` of its board (auto-detection would pick the same board
twice). ``/dev/serial/by-id/`` gives names that do not change when the
boards are plugged in a different order:

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_bringup motkin_dual_motor_testbed_ctrl.launch.py \
     namespace:=robot_a serial_port:=/dev/serial/by-id/<board_a>
   ros2 launch motkin_dual_motor_testbed_bringup motkin_dual_motor_testbed_ctrl.launch.py \
     namespace:=robot_b serial_port:=/dev/serial/by-id/<board_b>

The controllers of each testbed are then managed through
``/robot_a/controller_manager`` and ``/robot_b/controller_manager``, e.g.
``ros2 control list_controllers -c /robot_a/controller_manager``.

Checking that it runs
---------------------

In another terminal, with the workspace sourced:

.. code-block:: bash

   # Both controllers should be "active"
   ros2 control list_controllers

   # motor_1 and motor_2 should expose position, velocity, effort,
   # gain_kp and gain_kd command interfaces
   ros2 control list_hardware_interfaces

   # Joint states published by joint_state_broadcaster
   ros2 topic echo /joint_states

On the five-bar, ``/joint_states`` also contains ``passive_1`` and
``passive_2``. They have no encoder: the ``FiveBarSystem`` plugin of
``motkin_dual_motor_testbed_hardware`` computes them from ``motor_1`` and
``motor_2``, so the whole mechanism is displayed. The ``dual_flywheel`` model
only has the two motors.

In RViz, moving the motors by hand should move the model.

Stopping
--------

Press ``Ctrl+C`` in the launch terminal. Before stopping, send a zero command
(see :doc:`sending_commands`) so that the motors are not left with a
non-zero effort or gain.
