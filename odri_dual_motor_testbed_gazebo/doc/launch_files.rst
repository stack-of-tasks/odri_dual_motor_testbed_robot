Launch files
============

``odri_dual_motor_testbed_gazebo.launch.py``
--------------------------------------------

Starts the complete simulation. It:

#. sets ``GZ_SIM_RESOURCE_PATH`` so that Gazebo finds the meshes of
   ``odri_dual_motor_testbed_description``, and ``GZ_SIM_SYSTEM_PLUGIN_PATH``
   so that it finds ``FiveBarClosurePlugin`` and the ``odri_gz_ros2_control``
   plugin;
#. starts the Gazebo server on ``empty.sdf`` (running, 1 ms physics step)
   and, if ``gui`` is true, the Gazebo GUI with
   ``config/<robot_model>.config``;
#. runs xacro on
   ``odri_dual_motor_testbed_description/robots/<robot_model>_robot.urdf.xacro``
   with ``gz_sim:=true`` and
   ``controller_params_file:=config/forward_command_controller.yaml``;
#. starts ``robot_state_publisher`` and the ``/clock`` bridge
   (``ros_gz_bridge``), and sets ``use_sim_time`` to true;
#. spawns the robot through ``robot_spawn.launch.py``;
#. spawns ``joint_state_broadcaster`` and ``odri_forward_command_controller``.

.. code-block:: bash

   # Five-bar linkage (default)
   ros2 launch odri_dual_motor_testbed_gazebo odri_dual_motor_testbed_gazebo.launch.py

   # Dual flywheel, without the Gazebo GUI
   ros2 launch odri_dual_motor_testbed_gazebo odri_dual_motor_testbed_gazebo.launch.py \
     robot_model:=dual_flywheel gui:=false

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``robot_model``
     - ``fivebar_2dof``
     - Model to simulate: ``fivebar_2dof`` or ``dual_flywheel``.
   * - ``spawn_roll``
     - ``0``
     - Roll of the robot when it is spawned, in radians. With ``1.5708`` the
       five-bar stands vertically and gravity acts in its plane.
   * - ``gui``
     - ``true``
     - Start the Gazebo GUI client.

``robot_spawn.launch.py``
-------------------------

Spawns a robot description in a running Gazebo world with ``ros_gz_sim
create``. The description is passed as a string (``-string``) instead of
through the ``robot_description`` topic, to avoid missing a message published
before the subscriber was ready.

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``robot_name``
     - ``fivebar_2dof``
     - Name of the model in Gazebo.
   * - ``robot_description``
     - ``""``
     - URDF of the robot.
   * - ``x``, ``y``, ``z``
     - ``0.1``, ``0.0``, ``0.0``
     - Spawn position, in meters.
   * - ``roll``, ``pitch``, ``yaw``
     - ``0.0``
     - Spawn orientation, in radians.
