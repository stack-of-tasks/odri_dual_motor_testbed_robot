Packages
========

.. code-block:: text

   motkin_dual_motor_testbed_robot/          (repository)
   ├── motkin_dual_motor_testbed_robot/      This meta-package
   ├── motkin_dual_motor_testbed_description/
   ├── motkin_dual_motor_testbed_bringup/
   ├── motkin_dual_motor_testbed_hardware/
   ├── motkin_dual_motor_testbed_gazebo/
   ├── motkin_dual_motor_testbed_controllers/
   │   ├── motkin_forward_command_controller/
   │   ├── motkin_five_bar_force_velocity_controller/
   │   └── motkin_five_bar_force_velocity_py/
   ├── motkin_dual_motor_testbed_haptic_pair/
   └── five_bar_mgd_spec.md                Five-bar geometric model

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Package
     - Content
   * - ``motkin_dual_motor_testbed_description``
     - URDF/xacro models, meshes, ``ros2_control`` declarations for the real
       robot and for Gazebo, and a launch file to display the robot in RViz.
   * - ``motkin_dual_motor_testbed_bringup``
     - Launch files and controller configuration to start the real robot.
   * - ``motkin_dual_motor_testbed_hardware``
     - ``FiveBarSystem``, a ``ros2_control`` hardware plugin that wraps the
       motor board plugin and adds the passive joints of the five-bar,
       computed from the motor encoders.
   * - ``motkin_dual_motor_testbed_gazebo``
     - Launch file to simulate the robot in Gazebo Harmonic, and
       ``FiveBarClosurePlugin``, which closes the five-bar loop in
       simulation.
   * - ``motkin_forward_command_controller``
     - Controller that forwards position, velocity, effort, ``gain_kp`` and
       ``gain_kd`` commands to the motors from one topic. Used by default on
       the real robot and in simulation.
   * - ``motkin_five_bar_force_velocity_controller``
     - Controller that moves the five-bar end point in response to a
       contact force, q̇ = J\ :sup:`T` f\ :sub:`c`.
   * - ``motkin_five_bar_force_velocity_py``
     - The same control law as a Python node, on top of
       ``motkin_forward_command_controller``.
   * - ``motkin_dual_motor_testbed_haptic_pair``
     - Two simulated five-bars in one Gazebo world, a leader and a follower
       that reproduces the force applied to the leader.

Building the documentation
--------------------------

Each package is documented with rosdoc2. ``colcon build`` builds the
documentation of every package when rosdoc2 is available, and installs
it in ``install/<package>/share/<package>/doc``:

.. code-block:: bash

   sudo apt install python3-rosdoc2
   colcon build
   xdg-open install/motkin_dual_motor_testbed_robot/share/motkin_dual_motor_testbed_robot/doc/index.html
