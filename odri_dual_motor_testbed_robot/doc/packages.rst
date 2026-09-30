Packages
========

.. code-block:: text

   odri_dual_motor_testbed_robot/          (repository)
   ├── odri_dual_motor_testbed_robot/      This meta-package
   ├── odri_dual_motor_testbed_description/
   ├── odri_dual_motor_testbed_bringup/
   ├── odri_dual_motor_testbed_hardware/
   ├── odri_dual_motor_testbed_gazebo/
   ├── odri_dual_motor_testbed_controllers/
   │   ├── odri_forward_command_controller/
   │   ├── odri_five_bar_force_velocity_controller/
   │   └── odri_five_bar_force_velocity_py/
   ├── odri_dual_motor_testbed_haptic_pair/
   └── five_bar_mgd_spec.md                Five-bar geometric model

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Package
     - Content
   * - ``odri_dual_motor_testbed_description``
     - URDF/xacro models, meshes, ``ros2_control`` declarations for the real
       robot and for Gazebo, and a launch file to display the robot in RViz.
   * - ``odri_dual_motor_testbed_bringup``
     - Launch files and controller configuration to start the real robot.
   * - ``odri_dual_motor_testbed_hardware``
     - ``FiveBarSystem``, a ``ros2_control`` hardware plugin that wraps the
       motor board plugin and adds the passive joints of the five-bar,
       computed from the motor encoders.
   * - ``odri_dual_motor_testbed_gazebo``
     - Launch file to simulate the robot in Gazebo Harmonic, and
       ``FiveBarClosurePlugin``, which closes the five-bar loop in
       simulation.
   * - ``odri_forward_command_controller``
     - Controller that forwards position, velocity, effort, ``gain_kp`` and
       ``gain_kd`` commands to the motors from one topic. Used by default on
       the real robot and in simulation.
   * - ``odri_five_bar_force_velocity_controller``
     - Controller that moves the five-bar end point in response to a
       contact force, q̇ = J\ :sup:`T` f\ :sub:`c`.
   * - ``odri_five_bar_force_velocity_py``
     - The same control law as a Python node, on top of
       ``odri_forward_command_controller``.
   * - ``odri_dual_motor_testbed_haptic_pair``
     - Two simulated five-bars in one Gazebo world, a leader and a follower
       that reproduces the force applied to the leader.

Building the documentation
--------------------------

Each package is documented with rosdoc2. From the root of the repository:

.. code-block:: bash

   rosdoc2 build --package-path odri_dual_motor_testbed_robot
   rosdoc2 build --package-path odri_dual_motor_testbed_description
   rosdoc2 build --package-path odri_dual_motor_testbed_bringup
   rosdoc2 build --package-path odri_dual_motor_testbed_hardware
   rosdoc2 build --package-path odri_dual_motor_testbed_gazebo
   rosdoc2 build --package-path odri_dual_motor_testbed_controllers/odri_forward_command_controller
   rosdoc2 build --package-path odri_dual_motor_testbed_controllers/odri_five_bar_force_velocity_controller
   rosdoc2 build --package-path odri_dual_motor_testbed_controllers/odri_five_bar_force_velocity_py

The output is written to ``docs_output/<package>/index.html``.
