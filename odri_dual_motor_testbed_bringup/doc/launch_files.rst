Launch files
============

``odri_dual_motor_testbed_ctrl.launch.py``
------------------------------------------

Starts the robot on real hardware with ``joint_state_broadcaster``,
``odri_forward_command_controller``, ``robot_state_publisher`` and RViz (see
:doc:`starting_the_testbed`).

``odri_dual_motor_testbed.launch.py``
-------------------------------------

Same as ``odri_dual_motor_testbed_ctrl.launch.py`` without
``odri_forward_command_controller``: it only publishes the joint states and
displays the robot.

Arguments
---------

Both launch files take the same arguments:

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Argument
     - Default
     - Description
   * - ``robot_model``
     - ``fivebar_2dof``
     - Model to load: ``fivebar_2dof`` or ``dual_flywheel``.
   * - ``description_package``
     - ``odri_dual_motor_testbed_description``
     - Package providing ``robots/<robot_model>_robot.urdf.xacro``.
   * - ``runtime_config_package``
     - ``odri_dual_motor_testbed_bringup``
     - Package providing the controller configuration in ``config/``.
   * - ``controllers_file``
     - ``odri_dual_motor_testbed_controllers.yaml``
     - Controller configuration file.
   * - ``namespace``
     - ``""``
     - ROS namespace of this robot instance. The controller manager is
       ``/<namespace>/controller_manager`` and the topics, including ``tf``,
       are under ``/<namespace>``.
   * - ``serial_port``
     - ``""``
     - Serial device of the motor board, passed to the ``serial_port``
       xacro argument. Empty: auto-detected.

The robot description is built by running xacro on
``<description_package>/robots/<robot_model>_robot.urdf.xacro``, with the
default xacro arguments (``gz_sim:=false``, so the real hardware interface is
used). To simulate the testbed, use ``odri_dual_motor_testbed_gazebo``
instead.

Because the nodes can be namespaced, the controller YAML puts its entries
under the ``/**:`` wildcard key. A bare ``controller_manager:`` key only
matches a node in the root namespace.
