Control law
===========

Jacobian
--------

``include/motkin_five_bar_force_velocity_controller/five_bar_kinematics.hpp``
implements the direct geometric model of the five-bar (see
``five_bar_mgd_spec.md`` at the root of the repository): each motor θ\ :sub:`i`
drives a crank of length L\ :sub:`1` to an elbow E\ :sub:`i`, and both elbows
are connected to the end point P by couplers of length L\ :sub:`2`. P is the
intersection of two circles, computed in closed form.

The 2×2 velocity Jacobian J, with Ṗ = J θ̇, is obtained analytically by
differentiating the two closure constraints

.. math::

   \|P - E_L\|^2 = L_2^2, \qquad \|P - E_R\|^2 = L_2^2

It depends only on (θ\ :sub:`1`, θ\ :sub:`2`) and on the constant geometric
parameters. No kinematics library is used.
``test/test_five_bar_kinematics.cpp`` checks J against a central finite
difference of the geometric model.

Command
-------

The controller sends

.. math::

   \dot q = J^T f_c

with f\ :sub:`c` = (f\ :sub:`x`, f\ :sub:`y`) the contact force at P,
expressed in the (x, y) plane of ``case``. Using J\ :sup:`T` instead of
J\ :sup:`-T` or an admittance model is a deliberate simplification: it needs
no matrix inversion, has no singularity in the force path, and always moves
the joints in the direction that is power-conjugate to the applied force.

At each ``update()``
--------------------

#. Read the ``motor_1`` and ``motor_2`` positions (θ\ :sub:`1`,
   θ\ :sub:`2`).
#. Read the last message received on the contact force topic
   (``geometry_msgs/msg/WrenchStamped``). ``wrench.force.x`` and
   ``wrench.force.y`` are f\ :sub:`x` and f\ :sub:`y`; ``wrench.force.z``
   (out of plane) and the torques are ignored.
#. Compute q̇ = J\ :sup:`T` f\ :sub:`c` and clamp each component to
   ``max_joint_velocity``.
#. Write, on each motor:

   * ``position``: the measured position, so ``gain_kp`` only acts against
     drift;
   * ``velocity``: the component of q̇;
   * ``effort``: the gravity feed-forward (see below), or 0;
   * ``gain_kp`` and ``gain_kd``: the parameters of the same name.
     ``gain_kd`` is the gain that turns the velocity command into torque.

If the configuration is outside the workspace or the Jacobian is singular,
the controller logs a throttled warning and commands q̇ = 0.

Gravity compensation
--------------------

Gravity compensation is **disabled by default**. The five-bar moves in the
(x, y) plane of ``case`` with vertical motor axes, so gravity produces no
torque on the motors.

When ``gravity_compensation_enabled`` is true, the effort command is

.. math::

   \tau_{g,1} = m_{left}\, g\, (-L_1 \cos(\varphi_1 - \theta_1)), \qquad
   \tau_{g,2} = m_{right}\, g\, (-L_1 \cos(\varphi_2 - \theta_2))

This model assumes a vertically mounted five-bar whose coupler centres of
mass are at the elbows. Neither holds for the current mechanism (the centre
of mass of ``arm_l2`` is at (0.0487, 0, −0.002) in its frame), so it must be
derived again before it is used on a vertical mount.
