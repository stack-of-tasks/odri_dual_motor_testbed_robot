Running
=======

Prerequisites
-------------

The node needs a running testbed with ``joint_state_broadcaster`` and
``odri_forward_command_controller`` active, on the ``fivebar_2dof`` model:

.. code-block:: bash

   # Real robot
   ros2 launch odri_dual_motor_testbed_bringup odri_dual_motor_testbed_ctrl.launch.py

   # or simulation
   ros2 launch odri_dual_motor_testbed_gazebo odri_dual_motor_testbed_gazebo.launch.py

``force_velocity.launch.py``
----------------------------

Starts ``force_velocity_node`` under the name
``odri_five_bar_force_velocity_node``.

.. code-block:: bash

   ros2 launch odri_five_bar_force_velocity_py force_velocity.launch.py

.. list-table:: Arguments
   :header-rows: 1
   :widths: 35 25 40

   * - Argument
     - Default
     - Description
   * - ``forward_command_controller_name``
     - ``odri_forward_command_controller``
     - Controller to command. The default matches the bringup and Gazebo
       launch files of the testbed.
   * - ``contact_force_topic``
     - ``~/contact_force``
     - Contact force input topic.
   * - ``gain_kd``
     - ``0.05``
     - Velocity gain sent with each command.

The other parameters (see :doc:`node`) keep their defaults. To change them,
run the node directly:

.. code-block:: bash

   ros2 run odri_five_bar_force_velocity_py force_velocity_node --ros-args \
     -p max_joint_velocity:=1.0

Applying a force
----------------

Publish the force at P, in newtons, in the (x, y) plane of ``case``:

.. code-block:: bash

   # Push along +x
   ros2 topic pub -r 20 /odri_five_bar_force_velocity_node/contact_force \
     geometry_msgs/msg/WrenchStamped "{wrench: {force: {x: 1.0, y: 0.0, z: 0.0}}}"

   # Push along +y of case
   ros2 topic pub -r 20 /odri_five_bar_force_velocity_node/contact_force \
     geometry_msgs/msg/WrenchStamped "{wrench: {force: {x: 0.0, y: 1.0, z: 0.0}}}"

The node keeps using the last force received. Publish a zero force to stop
the motion.

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when ``BUILD_DOCS``
is enabled, and installs it in ``share/odri_five_bar_force_velocity_py/doc``:

.. code-block:: bash

   BUILD_DOCS=ON colcon build --packages-select odri_five_bar_force_velocity_py
   xdg-open install/odri_five_bar_force_velocity_py/share/odri_five_bar_force_velocity_py/doc/index.html

``--cmake-args -DBUILD_DOCS=ON`` has the same effect. The documentation is
only rebuilt when its sources change: ``doc/``, ``README.md``,
``package.xml``, and the headers or Python modules of the package.

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path odri_five_bar_force_velocity_py
   rosdoc2 open docs_output/odri_five_bar_force_velocity_py/index.html
