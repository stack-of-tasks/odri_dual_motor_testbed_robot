Package structure
=================

.. code-block:: text

   odri_dual_motor_testbed_description/
   ├── doc/            This documentation (rosdoc2)
   ├── gazebo/         Gazebo ros2_control system and plugins (gz_sim:=true)
   ├── launch/         show.launch.py, robot_state_publisher.launch.py
   ├── meshes/         STL meshes
   │   └── assets/
   │       ├── five_bar/        Meshes of the fivebar_2dof model
   │       └── dual_flywheel/   Meshes of the dual_flywheel model
   ├── robots/         Top-level xacro files, one per robot model
   ├── ros2_control/   ros2_control xacro macros (real hardware)
   ├── rviz/           RViz configuration
   └── urdf/           Kinematic trees included by the robots/ files

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when available,
and installs it in ``share/odri_dual_motor_testbed_description/doc``:

.. code-block:: bash

   colcon build --packages-select odri_dual_motor_testbed_description
   xdg-open install/odri_dual_motor_testbed_description/share/odri_dual_motor_testbed_description/doc/index.html

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path odri_dual_motor_testbed_description
   rosdoc2 open docs_output/odri_dual_motor_testbed_description/index.html

rosdoc2 builds every ``.rst`` and ``.md`` file in ``doc/`` automatically, so a
new page only has to be added to that directory.
