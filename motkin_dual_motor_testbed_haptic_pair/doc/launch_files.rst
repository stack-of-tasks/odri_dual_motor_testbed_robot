Launch files
============

``haptic_pair_gazebo.launch.py``
--------------------------------

Starts the Gazebo world, spawns the leader and the follower, each with
``joint_state_broadcaster`` and ``motkin_five_bar_force_velocity_controller``,
and starts ``contact_force_relay``.

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_haptic_pair haptic_pair_gazebo.launch.py

The Gazebo GUI uses ``config/haptic_pair.config`` by default, with the camera
set to show both robots. Use ``gui_config:=<file>`` to load another file, or
``gui:=false`` to run without a GUI.

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``controller_params_file``
     - ``config/haptic_pair_controllers.yaml``
     - controller_manager YAML loaded by both robot instances.
   * - ``leader_x``
     - ``-0.15``
     - X position of the leader, in meters.
   * - ``follower_x``
     - ``0.15``
     - X position of the follower, in meters.
   * - ``spawn_z``
     - ``0.01``
     - Z position of both robots, in meters.
   * - ``spawn_roll``
     - ``0``
     - Roll of both robots, in radians.
   * - ``gui``
     - ``true``
     - Start the Gazebo GUI client.
   * - ``gui_config``
     - ``config/haptic_pair.config``
     - Gazebo GUI configuration file.

``haptic_pair.launch.py``
-------------------------

Real-hardware counterpart of ``haptic_pair_gazebo.launch.py``, for two kits
plugged into the same computer. For each of the leader and the follower, it
starts a ``ros2_control_node``, a ``robot_state_publisher`` and the
``joint_state_broadcaster`` and ``motkin_forward_command_controller``
spawners under the robot's namespace, then starts ``position_coupling``
(see :doc:`architecture`). The real kits have no force sensor, so the
contact-force relay of the simulation is replaced by a position coupling
with force feedback estimated from the follower's motor current.

With two motkin boards connected, the serial port auto-detection of the
hardware interface would select the same board twice: the serial device of
each kit is therefore required. Use the stable ``/dev/serial/by-id/`` paths
(``ls /dev/serial/by-id/``) rather than ``/dev/ttyACM*``, whose numbering
depends on the plug order.

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_haptic_pair haptic_pair.launch.py \
     leader_serial_port:=/dev/serial/by-id/usb-...-leader \
     follower_serial_port:=/dev/serial/by-id/usb-...-follower

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``leader_serial_port``
     - (required)
     - Serial device of the leader's motkin board.
   * - ``follower_serial_port``
     - (required)
     - Serial device of the follower's motkin board.
   * - ``controller_params_file``
     - ``config/haptic_pair_hardware_controllers.yaml``
     - controller_manager YAML loaded by both robot instances.
   * - ``coupling_params_file``
     - ``config/position_coupling.yaml``
     - Gains and safety limits of ``position_coupling``.
   * - ``rviz``
     - ``false``
     - Open one RViz window per kit, each showing the robot of its namespace.

Driving the leader
------------------

On the real kits, move the leader by hand: the follower tracks it, and
blocking the follower is felt on the leader.

In simulation, publish a force on the leader's ``contact_force`` topic, from
a script, a joystick bridge, or Gazebo's own apply-force tool bridged to
ROS:

.. code-block:: bash

   ros2 topic pub /leader/motkin_five_bar_force_velocity_controller/contact_force \
     geometry_msgs/msg/WrenchStamped "{wrench: {force: {x: 1.0}}}"

Compare ``/leader/joint_states`` and ``/follower/joint_states`` to see the
follower reproduce the leader's motion.
