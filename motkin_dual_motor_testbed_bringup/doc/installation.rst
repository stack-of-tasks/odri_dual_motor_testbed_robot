Installation
============

Workspace
---------

The bringup package is part of the ``motkin_dual_motor_testbed_robot``
repository. Build it in a ROS 2 Jazzy workspace:

.. code-block:: bash

   mkdir -p motkin_dual_motor_testbed_ws/src
   cd motkin_dual_motor_testbed_ws/src
   git clone https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot.git
   cd ..
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source ./install/setup.bash

Dependencies
------------

At run time the bringup package needs:

* ``motkin_dual_motor_testbed_description``: robot models and the
  ``ros2_control`` hardware declaration;
* ``motkin_forward_command_controller``: the controller that forwards position,
  velocity, effort and gain commands to the motors. It lives in
  ``motkin_dual_motor_testbed_controllers/`` in the same repository;
* the Pico dual DRV8316C hardware interface
  (``motkin_ros2_hardware_interface``), loaded by the robot
  description when running on real hardware;
* ``motkin_dual_motor_testbed_hardware``: on the five-bar, its
  ``FiveBarSystem`` plugin wraps the Pico plugin and computes the passive
  joints ``passive_1`` and ``passive_2`` from the motor encoders;
* ``controller_manager``, ``robot_state_publisher``, ``rviz2`` and ``xacro``.

Hardware
--------

The motors are driven by a Raspberry Pi Pico dual DRV8316C board connected
over USB. Check that the board is visible before starting:

.. code-block:: bash

   ls /dev/serial/by-id/
   # usb-Raspberry_Pi_Pico_2_..._if00

The hardware interface finds the board by itself: it looks in
``/dev/serial/by-id``, then ``/dev/ttyACM*``, then ``/dev/ttyUSB*``. Your user
must be allowed to open the device, usually by being in the ``dialout``
group:

.. code-block:: bash

   sudo usermod -aG dialout $USER   # then log out and back in

The serial port, baud rate and timeout are parameters of the
``ros2_control`` system. They are described in the ``hardware_interfaces``
page of the ``motkin_dual_motor_testbed_description`` documentation.
