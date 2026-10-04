Configuration
=============

Parameters
----------

The parameters are declared with ``generate_parameter_library`` in
``src/odri_five_bar_force_velocity_controller_parameters.yaml``.

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Parameter
     - Default
     - Description
   * - ``motor_left_joint``
     - ``motor_1``
     - Left actuated joint (θ\ :sub:`1`). Read-only.
   * - ``motor_right_joint``
     - ``motor_2``
     - Right actuated joint (θ\ :sub:`2`). Read-only.
   * - ``contact_force_topic``
     - ``~/contact_force``
     - ``geometry_msgs/msg/WrenchStamped`` topic giving the force at P.
       Read-only.
   * - ``gain_kp``
     - ``0.0``
     - Position gain sent with each command. The position setpoint is the
       measured position, so this only acts against drift.
   * - ``gain_kd``
     - ``0.05``
     - Velocity gain sent with each command.
   * - ``max_joint_velocity``
     - ``2.0``
     - Clamp on each component of q̇, in rad/s.
   * - ``gravity_compensation_enabled``
     - ``false``
     - Send the gravity feed-forward as effort instead of 0.
   * - ``gravity``
     - ``9.81``
     - Gravitational acceleration, in m/s².
   * - ``coupler_mass_left``
     - ``0.00836663``
     - Mass of ``arm_l2``, in kg.
   * - ``coupler_mass_right``
     - ``0.00968954``
     - Mass of ``arm_r2``, in kg.
   * - ``geometry.a_x``, ``geometry.a_z``
     - ``-0.0421006``, ``0.0302628``
     - Axis of ``motor_1`` in the (x, y) plane of ``case``, in meters.
       ``a_z`` holds the y coordinate: the name is historical.
   * - ``geometry.b_x``, ``geometry.b_z``
     - ``0.0578994``, ``0.0302628``
     - Axis of ``motor_2``, same convention.
   * - ``geometry.l1``, ``geometry.l2``
     - ``0.06``, ``0.1``
     - Crank and coupler lengths, in meters.
   * - ``geometry.phi1``, ``geometry.phi2``
     - ``1.5707963267948966``
     - Angular offsets of the cranks, in radians.

The ``geometry`` defaults are those of the ``fivebar_2dof`` model and only
need to be changed if the mechanism changes.

Example
-------

.. code-block:: yaml

   controller_manager:
     ros__parameters:
       update_rate: 100  # Hz
       odri_five_bar_fv:
         type: odri_five_bar_force_velocity_controller/OdriFiveBarForceVelocityController

   odri_five_bar_fv:
     ros__parameters:
       motor_left_joint: motor_1
       motor_right_joint: motor_2
       gain_kp: 0.0
       gain_kd: 0.05
       max_joint_velocity: 2.0
       gravity_compensation_enabled: false

With this configuration the contact force topic is
``/odri_five_bar_fv/contact_force``.

Building and testing
--------------------

.. code-block:: bash

   colcon build --packages-select odri_five_bar_force_velocity_controller
   colcon test --packages-select odri_five_bar_force_velocity_controller
   colcon test-result --verbose

``test/test_five_bar_kinematics.cpp`` checks the Jacobian and the gravity
torque against finite differences.

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when available,
and installs it in ``share/odri_five_bar_force_velocity_controller/doc``:

.. code-block:: bash

   colcon build --packages-select odri_five_bar_force_velocity_controller
   xdg-open install/odri_five_bar_force_velocity_controller/share/odri_five_bar_force_velocity_controller/doc/index.html

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path odri_five_bar_force_velocity_controller
   rosdoc2 open docs_output/odri_five_bar_force_velocity_controller/index.html
