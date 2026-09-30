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

Run from the directory that contains this package:

.. code-block:: bash

   rosdoc2 build --package-path odri_dual_motor_testbed_hardware
   rosdoc2 open docs_output/odri_dual_motor_testbed_hardware/index.html
