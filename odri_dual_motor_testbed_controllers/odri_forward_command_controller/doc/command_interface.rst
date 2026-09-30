Command interface
=================

Topic
-----

The controller subscribes to ``~/commands``
(``std_msgs/msg/Float64MultiArray``). With the configuration of the testbed,
the topic is ``/odri_forward_command_controller/commands``.

For **n** joints, ``data`` must hold exactly **5 × n** values, grouped by
interface, in the order of the ``joints`` parameter:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Slice
     - Content
   * - ``data[0 .. n)``
     - ``position`` commands (rad)
   * - ``data[n .. 2n)``
     - ``velocity`` commands (rad/s)
   * - ``data[2n .. 3n)``
     - ``effort`` commands (N·m)
   * - ``data[3n .. 4n)``
     - ``gain_kp`` (proportional gain)
   * - ``data[4n .. 5n)``
     - ``gain_kd`` (derivative gain)

* A message with another number of values is ignored.
* A ``NaN`` value leaves the corresponding command unchanged. This allows
  partial updates, for example changing the positions without touching the
  gains.

Claimed interfaces
------------------

For each joint of ``joints``, the controller claims the ``position``,
``velocity``, ``effort``, ``gain_kp`` and ``gain_kd`` command interfaces. It
claims no state interface.

Examples
--------

With ``joints: [motor_1, motor_2]``:

.. code-block:: bash

   # Hold both motors at 0 with a PD
   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0,   0.0, 0.0,   0.0, 0.0,   5.0, 5.0,   0.1, 0.1]}"
   #         ^pos×2       ^vel×2      ^eff×2      ^kp×2       ^kd×2

   # Move to new positions, keep the gains
   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.5, -0.5,  0.0, 0.0,  0.0, 0.0,  .nan, .nan,  .nan, .nan]}"

   # Zero command: no torque
   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

From Python:

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float64MultiArray

   rclpy.init()
   node = Node("odri_command_example")
   pub = node.create_publisher(
       Float64MultiArray, "/odri_forward_command_controller/commands", 10)

   positions = [0.1, -0.2]   # rad
   velocities = [0.0, 0.0]   # rad/s
   efforts = [0.0, 0.0]      # N.m
   gains_kp = [5.0, 5.0]
   gains_kd = [0.1, 0.1]

   msg = Float64MultiArray()
   msg.data = positions + velocities + efforts + gains_kp + gains_kd
   pub.publish(msg)

.. note::

   The hardware interface limits each command. See the
   ``hardware_interfaces`` page of the ``odri_dual_motor_testbed_description``
   documentation for the ranges on the real robot.
