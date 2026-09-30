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

The robot description is built by running xacro on
``<description_package>/robots/<robot_model>_robot.urdf.xacro``, with the
default xacro arguments (``gz_sim:=false``, so the real hardware interface is
used). To simulate the testbed, use ``odri_dual_motor_testbed_gazebo``
instead.

Other launch files
------------------

``odri_dual_motor_testbed_rviz.launch.py``,
``odri_dual_motor_testbed_pub.launch.py``,
``odri_dual_motor_testbed_backup.launch.py``,
``odri_dual_motor_testbed_position_only.launch.py`` and
``test_forward_position_controller.launch.py`` are older launch files and do
not work in their current state. The first four refer to
``odri_dual_motor_testbed.urdf.xacro``, which is no longer in the description
package; the last one uses a configuration file from
``ros2_control_bolt_bringup``.
