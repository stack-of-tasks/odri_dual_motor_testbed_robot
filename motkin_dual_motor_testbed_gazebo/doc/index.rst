motkin_dual_motor_testbed_gazebo
==============================

Simulation of the MOTKIN dual motor testbed in Gazebo Harmonic (gz-sim 8) with
``ros2_control``.

The package provides:

* a launch file that starts Gazebo, spawns the robot, and loads the same
  controllers as on the real hardware;
* the ``FiveBarClosurePlugin``, a Gazebo system plugin that closes the
  kinematic loop of the five-bar linkage;
* the controller configuration and the Gazebo GUI layouts used in simulation.

The robot models and the Gazebo ``ros2_control`` declarations come from
``motkin_dual_motor_testbed_description``. The ``ros2_control`` hardware
interface used in Gazebo is ``GazeboMotkinSimSystem``, from the
``motkin_gz_ros2_control`` package.

Quick start:

.. code-block:: bash

   ros2 launch motkin_dual_motor_testbed_gazebo motkin_dual_motor_testbed_gazebo.launch.py robot_model:=fivebar_2dof

.. toctree::
   :maxdepth: 2
   :caption: Contents

   package_structure
   launch_files
   simulated_control
   five_bar_closure_plugin
