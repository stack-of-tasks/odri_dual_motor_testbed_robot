Launch files
============

``haptic_pair.launch.py``
-------------------------

Starts the Gazebo world, spawns the leader and the follower, each with
``joint_state_broadcaster`` and ``motkin_five_bar_force_velocity_controller``,
and starts ``contact_force_relay``.

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_haptic_pair haptic_pair.launch.py

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

Driving the leader
------------------

Publish a force on the leader's ``contact_force`` topic, from a script, a
joystick bridge, or Gazebo's own apply-force tool bridged to ROS:

.. code-block:: bash

   ros2 topic pub /leader/motkin_five_bar_force_velocity_controller/contact_force \
     geometry_msgs/msg/WrenchStamped "{wrench: {force: {x: 1.0}}}"

Compare ``/leader/joint_states`` and ``/follower/joint_states`` to see the
follower reproduce the leader's motion.
