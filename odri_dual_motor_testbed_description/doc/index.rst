odri_dual_motor_testbed_description
===================================

URDF/xacro description of the ODRI dual motor testbed, with meshes, the
``ros2_control`` hardware declarations and launch files to display it in RViz
or simulate it in Gazebo.

Two robot models are provided:

- ``fivebar_2dof``: a planar two-degree-of-freedom five-bar linkage (default);
- ``dual_flywheel``: one flywheel on each motor.

Quick start:

.. code-block:: bash

   ros2 launch odri_dual_motor_testbed_description show.launch.py robot_model:=fivebar_2dof
   ros2 launch odri_dual_motor_testbed_description show.launch.py robot_model:=dual_flywheel

.. toctree::
   :maxdepth: 2
   :caption: Contents

   package_structure
   robot_models
   hardware_interfaces
   launch_files
