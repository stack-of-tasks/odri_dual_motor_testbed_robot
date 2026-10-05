Launch files
============

``motkin_dual_motor_testbed_gazebo.launch.py``
----------------------------------------------

Starts the complete simulation of one robot, in the root namespace. It
includes ``gz_world.launch.py`` (with ``config/<robot_model>.config`` as GUI
configuration) and then ``robot_spawn.launch.py`` once, which loads
``config/forward_command_controller.yaml`` and spawns
``joint_state_broadcaster`` and ``motkin_forward_command_controller``.

.. code-block:: bash

   # Five-bar linkage (default)
   ros2 launch motkin_dual_motor_testbed_gazebo motkin_dual_motor_testbed_gazebo.launch.py

   # Dual flywheel, without the Gazebo GUI
   ros2 launch motkin_dual_motor_testbed_gazebo motkin_dual_motor_testbed_gazebo.launch.py \
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

``gz_world.launch.py``
----------------------

Starts an empty world ready to receive robots. It:

#. sets ``GZ_SIM_RESOURCE_PATH`` so that Gazebo finds the meshes of
   ``motkin_dual_motor_testbed_description``, and ``GZ_SIM_SYSTEM_PLUGIN_PATH``
   so that it finds ``FiveBarClosurePlugin`` and the ``motkin_gz_ros2_control``
   plugin;
#. starts the Gazebo server on ``empty.sdf`` (running, 1 ms physics step)
   and, if ``gui`` is true, the Gazebo GUI with ``gui_config``;
#. starts the ``/clock`` bridge (``ros_gz_bridge``).

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``gui``
     - ``true``
     - Start the Gazebo GUI client.
   * - ``gui_config``
     - ``config/gui.config``
     - Gazebo GUI configuration file.

``robot_spawn.launch.py``
-------------------------

Adds one robot to a running world (started by ``gz_world.launch.py``). It
can be included several times with different ``namespace`` values to
simulate several robots in the same world, as
``motkin_dual_motor_testbed_haptic_pair`` does. It:

#. runs xacro on
   ``motkin_dual_motor_testbed_description/robots/<robot_model>_robot.urdf.xacro``
   with ``gz_sim:=true``, ``robot_namespace:=<namespace>`` and
   ``controller_params_file:=<controller_params_file>``. The namespace ends up
   in the ``motkin_gz_ros2_control`` plugin, so the robot's controller manager is
   ``/<namespace>/controller_manager``;
#. starts ``robot_state_publisher`` in ``<namespace>``;
#. spawns the model with ``ros_gz_sim create``. The description is passed as
   a string (``-string``) instead of through the ``robot_description`` topic,
   to avoid missing a message published before the subscriber was ready;
#. spawns ``joint_state_broadcaster`` and every controller in
   ``controllers`` on ``/<namespace>/controller_manager``.

Because the nodes are namespaced, the controller YAML must put its entries
under the ``/**:`` wildcard key. A bare ``controller_manager:`` key only
matches a node in the root namespace.

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_gazebo robot_spawn.launch.py \
     namespace:=robot_a x:=-0.3

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 20 55

   * - Argument
     - Default
     - Description
   * - ``namespace``
     - ``""``
     - ROS namespace of this robot instance.
   * - ``robot_model``
     - ``fivebar_2dof``
     - ``fivebar_2dof`` or ``dual_flywheel``.
   * - ``robot_name``
     - ``""``
     - Name of the model in Gazebo. Must be unique in the world. If it is
       empty, it is ``<namespace>_<robot_model>``, or ``<robot_model>``
       without a namespace.
   * - ``controller_params_file``
     - ``config/forward_command_controller.yaml``
     - controller_manager YAML loaded by ``motkin_gz_ros2_control``.
   * - ``controllers``
     - ``motkin_forward_command_controller``
     - Controllers to spawn after ``joint_state_broadcaster``, separated by
       spaces.
   * - ``x``, ``y``, ``z``
     - ``0.1``, ``0.0``, ``0.0``
     - Spawn position, in meters.
   * - ``roll``, ``pitch``, ``yaw``
     - ``0.0``
     - Spawn orientation, in radians.
