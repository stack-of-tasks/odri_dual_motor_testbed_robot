Package structure
=================

.. code-block:: text

   motkin_dual_motor_testbed_haptic_pair/
   ├── config/
   │   ├── haptic_pair_controllers.yaml   controller_manager configuration (simulation)
   │   ├── haptic_pair_hardware_controllers.yaml  controller_manager configuration (real kits)
   │   ├── position_coupling.yaml         position_coupling gains (real kits)
   │   └── haptic_pair.config             Gazebo GUI layout showing both robots
   ├── doc/            This documentation (rosdoc2)
   ├── launch/
   │   ├── haptic_pair_gazebo.launch.py   Simulation entry point (Gazebo)
   │   └── haptic_pair.launch.py          Two real kits on the same computer
   ├── motkin_dual_motor_testbed_haptic_pair/
   │   ├── contact_force_relay.py         Leader -> follower force relay node (simulation)
   │   └── position_coupling.py           Position forward / current feedback node (real kits)
   └── CMakeLists.txt  ament_cmake + ament_cmake_python build

Building
--------

.. code-block:: bash

   colcon build --packages-select motkin_dual_motor_testbed_haptic_pair
   source install/setup.bash

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when ``BUILD_DOCS``
is enabled, and installs it in
``share/motkin_dual_motor_testbed_haptic_pair/doc``:

.. code-block:: bash

   BUILD_DOCS=ON colcon build --packages-select motkin_dual_motor_testbed_haptic_pair
   xdg-open install/motkin_dual_motor_testbed_haptic_pair/share/motkin_dual_motor_testbed_haptic_pair/doc/index.html

``--cmake-args -DBUILD_DOCS=ON`` has the same effect. The documentation is
only rebuilt when its sources change: ``doc/``, ``README.md``,
``package.xml``, and the Python modules of the package.

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path motkin_dual_motor_testbed_haptic_pair
   rosdoc2 open docs_output/motkin_dual_motor_testbed_haptic_pair/index.html
