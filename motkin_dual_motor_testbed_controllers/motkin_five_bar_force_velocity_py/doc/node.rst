The force_velocity_node
=======================

Control law
-----------

``motkin_five_bar_force_velocity_py/five_bar_kinematics.py`` implements the
direct geometric model of the five-bar and its 2×2 velocity Jacobian J
(Ṗ = J θ̇), derived analytically from the two closure constraints
‖P − E\ :sub:`L`‖² = L\ :sub:`2`\ ² and ‖P − E\ :sub:`R`‖² =
L\ :sub:`2`\ ². It is the Python version of the model used by
``motkin_five_bar_force_velocity_controller``, whose documentation gives the
details. The derivation of the geometric model is in ``five_bar_mgd_spec.md``
at the root of the repository.

At ``control_rate``, the node computes q̇ = J\ :sup:`T` f\ :sub:`c`, clamps
each component to ``max_joint_velocity`` and publishes the command. If the
configuration is outside the workspace or the Jacobian is singular, it logs a
throttled warning and publishes nothing for that cycle.

Topics
------

.. list-table::
   :header-rows: 1
   :widths: 35 30 35

   * - Topic
     - Type
     - Use
   * - ``/joint_states``
     - ``sensor_msgs/msg/JointState``
     - Subscribed. Positions of ``motor_left_joint`` and
       ``motor_right_joint`` (θ\ :sub:`1`, θ\ :sub:`2`).
   * - ``contact_force_topic`` (``~/contact_force``)
     - ``geometry_msgs/msg/WrenchStamped``
     - Subscribed. Contact force at P, in the (x, y) plane of ``case``:
       ``wrench.force.x`` and ``wrench.force.y``. ``wrench.force.z`` and the
       torques are ignored.
   * - ``/<forward_command_controller_name>/commands``
     - ``std_msgs/msg/Float64MultiArray``
     - Published. Command of ``motkin_forward_command_controller``.

Each command has the layout of ``motkin_forward_command_controller``:

.. code-block:: text

   [NaN, NaN,  q̇1, q̇2,  τg1, τg2,  0.0, 0.0,  gain_kd, gain_kd]
    position   velocity  effort    gain_kp   gain_kd

* ``position`` is ``NaN``, so the controller keeps its previous position
  command;
* ``effort`` is the gravity feed-forward, or 0 when gravity compensation is
  disabled. It is always an explicit number: with ``NaN``, a previous effort
  would stay active;
* ``gain_kp`` is 0 and ``gain_kd`` is the parameter, so the motor torque is
  ``gain_kd`` × (q̇ − measured velocity), plus the effort.

Gravity compensation is disabled by default because the five-bar moves in a
horizontal plane. See the ``control_law`` page of
``motkin_five_bar_force_velocity_controller`` for the model and its limits.

Parameters
----------

.. list-table::
   :header-rows: 1
   :widths: 35 20 45

   * - Parameter
     - Default
     - Description
   * - ``motor_left_joint``
     - ``motor_1``
     - Left actuated joint (θ\ :sub:`1`).
   * - ``motor_right_joint``
     - ``motor_2``
     - Right actuated joint (θ\ :sub:`2`).
   * - ``forward_command_controller_name``
     - ``motkin_forward_command_controller``
     - Name of the ``motkin_forward_command_controller`` instance to command,
       as started by the bringup and Gazebo launch files.
   * - ``contact_force_topic``
     - ``~/contact_force``
     - Contact force input topic.
   * - ``gain_kd``
     - ``0.05``
     - Velocity gain sent with each command.
   * - ``max_joint_velocity``
     - ``2.0``
     - Clamp on each component of q̇, in rad/s.
   * - ``control_rate``
     - ``100.0``
     - Rate of the control loop, in Hz.
   * - ``gravity_compensation_enabled``
     - ``false``
     - Send the gravity feed-forward as effort instead of 0.
   * - ``gravity``
     - ``9.81``
     - Gravitational acceleration, in m/s².
   * - ``coupler_mass_left``, ``coupler_mass_right``
     - ``0.00836663``, ``0.00968954``
     - Masses of ``arm_l2`` and ``arm_r2``, in kg.
   * - ``geometry.a_x``, ``geometry.a_z``, ``geometry.b_x``,
       ``geometry.b_z``, ``geometry.l1``, ``geometry.l2``,
       ``geometry.phi1``, ``geometry.phi2``
     - ``fivebar_2dof`` values
     - Geometry of the five-bar. ``a_z`` and ``b_z`` hold the y
       coordinates: the names are historical.
