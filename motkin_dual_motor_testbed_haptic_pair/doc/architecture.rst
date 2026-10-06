Architecture
============

Simulation
----------

The launch file starts one world with ``motkin_dual_motor_testbed_gazebo``'s
``gz_world.launch.py``, then includes ``motkin_dual_motor_testbed_gazebo``'s
``robot_spawn.launch.py`` twice, with ``namespace:=leader`` and
``namespace:=follower``. Each robot gets its own ``controller_manager``
(``/leader/controller_manager``, ``/follower/controller_manager``), which
loads the *existing* ``motkin_five_bar_force_velocity_controller`` (from
``motkin_five_bar_force_velocity_controller``). That controller already
implements the admittance law q̇ = J\ :sup:`T` f\ :sub:`c`, which turns a
sensed contact force into motion for this mechanism.

The only new code is ``contact_force_relay``: it republishes whatever
``geometry_msgs/WrenchStamped`` the leader receives on
``/leader/motkin_five_bar_force_velocity_controller/contact_force`` onto
``/follower/motkin_five_bar_force_velocity_controller/contact_force``. Since the
follower runs the identical controller, it reproduces the same force -- and,
by construction of the shared admittance law, the same motion -- with no new
control algorithm.

.. code-block:: text

   [external force] -> leader/contact_force -> leader controller -> leader motion
                               |
                        contact_force_relay
                               v
                        follower/contact_force -> follower controller -> follower motion

``contact_force_relay`` node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Parameters
   :header-rows: 1
   :widths: 25 45 30

   * - Parameter
     - Default
     - Description
   * - ``input_topic``
     - ``/leader/motkin_five_bar_force_velocity_controller/contact_force``
     - Topic the force is read from.
   * - ``output_topic``
     - ``/follower/motkin_five_bar_force_velocity_controller/contact_force``
     - Topic the force is republished on.

Real kits
---------

The real kits have no force sensor, so nothing can be published on the
leader's ``contact_force`` topic when a user pushes it by hand. Estimating
that force from the leader's own motor current does not help either: the
leader current is the controller's own reaction, iq = kd (v\ :sub:`target` - v),
and feeding it back into v\ :sub:`target` only rescales the leader damping,
so the follower would not move at the leader's velocity.

``haptic_pair.launch.py`` therefore uses the classical position-forward /
force-feedback teleoperation scheme. Both kits run
``motkin_forward_command_controller``, and the ``position_coupling`` node
drives them through the firmware law
iq = iff + kp (q\ :sub:`target` - q) + kd (v\ :sub:`target` - v):

* follower: q\ :sub:`target` = q\ :sub:`leader`, v\ :sub:`target` = v\ :sub:`leader`;
* leader: the force met by the follower is estimated from its measured
  current, low-pass filtered, and applied as feed-forward current,
  iff\ :sub:`leader` = -g i\ :sub:`follower`. The kits share the same
  kinematics and, once coupled, the same joint angles, so the Jacobians of
  the force -> torque mappings cancel and no Jacobian is needed.

.. code-block:: text

   [hand] -> leader q, v --------------------------> follower PD -> follower motion
                ^                                          |
                |                                   follower current i_F
                +--- iff = -g * lowpass(i_F) <-------------+

``position_coupling`` node
^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameters (``config/position_coupling.yaml``; gains in A/rad and A.s/rad):

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Parameter
     - Default
     - Description
   * - ``follower_kp`` / ``follower_kd``
     - 2.0 / 0.05
     - PD of the follower towards the leader.
   * - ``force_feedback_gain``
     - 0.5
     - g: leader feed-forward current per follower ampere (0 disables).
   * - ``force_feedback_cutoff``
     - 10.0
     - Cut-off frequency (Hz) of the low-pass filter on the follower current.
   * - ``max_feedback_current``
     - 0.5
     - Clamp (A) on the leader feed-forward current.
   * - ``leader_kp`` / ``leader_kd``
     - 0.0 / 0.0
     - Optional PD of the leader towards the follower.
   * - ``max_position_error``
     - 0.3
     - Clamp (rad) on the position error of each target.
   * - ``state_timeout``
     - 0.1
     - Both kits are released (zero gains) when their joint states are older
       than this (s).
