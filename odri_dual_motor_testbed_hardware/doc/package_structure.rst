Package structure
=================

.. code-block:: text

   odri_dual_motor_testbed_hardware/
   ├── doc/            This documentation (rosdoc2)
   ├── include/odri_dual_motor_testbed_hardware/
   │   ├── five_bar_system.hpp          FiveBarSystem hardware plugin
   │   └── five_bar_passive_joints.hpp  Direct geometric model (header only)
   ├── src/
   │   └── five_bar_system.cpp
   ├── test/
   │   └── test_five_bar_passive_joints.cpp
   └── odri_dual_motor_testbed_hardware.xml   pluginlib description

The plugin is exported to pluginlib as
``odri_dual_motor_testbed_hardware/FiveBarSystem``, with base class
``hardware_interface::SystemInterface``.

Building and testing
--------------------

.. code-block:: bash

   colcon build --packages-select odri_dual_motor_testbed_hardware
   colcon test --packages-select odri_dual_motor_testbed_hardware
   colcon test-result --verbose

The default wrapped plugin comes from
``pico_dual_drv8316c_ros2_hardware_interface``. It is only needed at run
time, when the plugin is loaded.

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when ``BUILD_DOCS``
is enabled, and installs it in ``share/odri_dual_motor_testbed_hardware/doc``:

.. code-block:: bash

   BUILD_DOCS=ON colcon build --packages-select odri_dual_motor_testbed_hardware
   xdg-open install/odri_dual_motor_testbed_hardware/share/odri_dual_motor_testbed_hardware/doc/index.html

``--cmake-args -DBUILD_DOCS=ON`` has the same effect. The documentation is
only rebuilt when its sources change: ``doc/``, ``README.md``,
``package.xml``, and the headers or Python modules of the package.

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path odri_dual_motor_testbed_hardware
   rosdoc2 open docs_output/odri_dual_motor_testbed_hardware/index.html
