Package structure
=================

.. code-block:: text

   motkin_dual_motor_testbed_gazebo/
   ├── config/
   │   ├── forward_command_controller.yaml   controller_manager configuration
   │   ├── fivebar_2dof.config               Gazebo GUI layout, five-bar
   │   ├── dual_flywheel.config              Gazebo GUI layout, flywheels
   │   └── gui.config                        Generic Gazebo GUI layout
   ├── doc/            This documentation (rosdoc2)
   ├── launch/
   │   ├── motkin_dual_motor_testbed_gazebo.launch.py   Main entry point (one robot)
   │   ├── gz_world.launch.py                         Starts Gazebo and the /clock bridge
   │   └── robot_spawn.launch.py                      Spawns one (namespaced) robot
   └── src/
       ├── FiveBarClosurePlugin.hh   Loop closure plugin (gz-sim 8)
       └── FiveBarClosurePlugin.cc

The plugin is built as ``libFiveBarClosurePlugin.so`` and installed in
``lib/``. The launch file adds that directory, and the one of
``motkin_gz_ros2_control``, to ``GZ_SIM_SYSTEM_PLUGIN_PATH``.

Building
--------

.. code-block:: bash

   colcon build --packages-select motkin_dual_motor_testbed_gazebo
   source install/setup.bash

The plugin needs ``gz-sim8`` and ``gz-plugin2``. On ROS 2 Jazzy they come from
the ``gz_sim_vendor`` and ``gz_plugin_vendor`` packages.

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when available,
and installs it in ``share/motkin_dual_motor_testbed_gazebo/doc``:

.. code-block:: bash

   colcon build --packages-select motkin_dual_motor_testbed_gazebo
   xdg-open install/motkin_dual_motor_testbed_gazebo/share/motkin_dual_motor_testbed_gazebo/doc/index.html

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path motkin_dual_motor_testbed_gazebo
   rosdoc2 open docs_output/motkin_dual_motor_testbed_gazebo/index.html
