Package structure
=================

.. code-block:: text

   odri_dual_motor_testbed_gazebo/
   ├── config/
   │   ├── forward_command_controller.yaml   controller_manager configuration
   │   ├── fivebar_2dof.config               Gazebo GUI layout, five-bar
   │   ├── dual_flywheel.config              Gazebo GUI layout, flywheels
   │   └── gui.config                        Generic Gazebo GUI layout
   ├── doc/            This documentation (rosdoc2)
   ├── launch/
   │   ├── odri_dual_motor_testbed_gazebo.launch.py   Main entry point
   │   └── robot_spawn.launch.py                      Spawns a robot description
   └── src/
       ├── FiveBarClosurePlugin.hh   Loop closure plugin (gz-sim 8)
       └── FiveBarClosurePlugin.cc

The plugin is built as ``libFiveBarClosurePlugin.so`` and installed in
``lib/``. The launch file adds that directory, and the one of
``odri_gz_ros2_control``, to ``GZ_SIM_SYSTEM_PLUGIN_PATH``.

Building
--------

.. code-block:: bash

   colcon build --packages-select odri_dual_motor_testbed_gazebo
   source install/setup.bash

The plugin needs ``gz-sim8`` and ``gz-plugin2``. On ROS 2 Jazzy they come from
the ``gz_sim_vendor`` and ``gz_plugin_vendor`` packages.

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when ``BUILD_DOCS``
is enabled, and installs it in ``share/odri_dual_motor_testbed_gazebo/doc``:

.. code-block:: bash

   BUILD_DOCS=ON colcon build --packages-select odri_dual_motor_testbed_gazebo
   xdg-open install/odri_dual_motor_testbed_gazebo/share/odri_dual_motor_testbed_gazebo/doc/index.html

``--cmake-args -DBUILD_DOCS=ON`` has the same effect. The documentation is
only rebuilt when its sources change: ``doc/``, ``README.md``,
``package.xml``, and the headers or Python modules of the package.

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path odri_dual_motor_testbed_gazebo
   rosdoc2 open docs_output/odri_dual_motor_testbed_gazebo/index.html
