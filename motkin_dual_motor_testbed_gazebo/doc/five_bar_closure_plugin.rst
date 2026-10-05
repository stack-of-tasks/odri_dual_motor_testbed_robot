Five-bar closure plugin
=======================

URDF only describes kinematic trees, so the five-bar is loaded in Gazebo as
two open chains:

.. code-block:: text

   case ─(motor_1)─ rotor   ─(passive_1)─ arm_l2 ── P1
   case ─(motor_2)─ rotor_2 ─(passive_2)─ arm_r2 ── P2

``FiveBarClosurePlugin`` (``motkin_gz::FiveBarClosurePlugin``) joins the two
arm tips P\ :sub:`1` and P\ :sub:`2` at the closure point P with a planar pin
joint. It never writes the passive joints. Instead, it computes the force
transmitted through P and applies it to both arms, so the passive joints and
the position of P follow from the simulated dynamics.

The plugin is declared in
``motkin_dual_motor_testbed_description/gazebo/gazebo_fivebar_2dof.urdf.xacro``.
It is only used by the ``fivebar_2dof`` model.

Constraint force
----------------

The constraint is

.. math::

   c(q) = P_1(\theta_1, \beta_1) - P_2(\theta_2, \beta_2) = 0

restricted to the plane of the mechanism, which is normal to the motor axes.
The out-of-plane components are carried by the revolute joints.

At every step, the plugin computes the in-plane force λ transmitted at P, the
Lagrange multiplier of the constraint:

.. math::

   \left(J M^{-1} J^T\right) \lambda =
   -\frac{\beta}{\Delta t^2} c - \frac{1}{\Delta t} J \dot q - J \ddot q_{free}

and applies +λ at P\ :sub:`1` on ``arm_l2`` and −λ at P\ :sub:`2` on
``arm_r2``. The terms are:

* J, the Jacobian of P\ :sub:`1` − P\ :sub:`2` with respect to the four joint
  angles (θ\ :sub:`1`, β\ :sub:`1`, θ\ :sub:`2`, β\ :sub:`2`);
* M, the joint-space mass matrix of each arm, built from the link inertias
  that Gazebo holds;
* q̈\ :sub:`free`, the joint acceleration without the constraint. It is
  estimated from the previous step: the measured acceleration minus the part
  caused by the last λ. This takes the motor torques, damping, Coriolis terms
  and contacts into account without modelling them;
* β, the Baumgarte factor: the fraction of the closure error removed at each
  step.

After the step, the constraint velocity is J q̇ = −β c / Δt, so any drift of
the loop decays geometrically.

Start-up
--------

Gazebo spawns the passive joints at 0, which does not close the loop. At the
first step, the plugin computes the passive joint angles with the exact
direct geometric model and resets ``passive_1`` and ``passive_2`` once, so the
simulation starts with a closed loop. Set ``initialize_with_mgd`` to false to
skip this. The loop is then pulled closed by the constraint force, which is
clamped to ``max_force``.

The derivation of the direct geometric model is in ``five_bar_mgd_spec.md``
at the root of the repository.

Parameters
----------

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - SDF element
     - Five-bar value
     - Description
   * - ``motor_joint1``, ``motor_joint2``
     - ``motor_1``, ``motor_2``
     - Actuated joints.
   * - ``elbow_joint1``, ``elbow_joint2``
     - ``passive_1``, ``passive_2``
     - Passive joints. Their child links carry the closure point.
   * - ``tip_offset1``
     - ``0.1 0 0``
     - Position of P in the ``arm_l2`` frame (origin of
       ``closing_tip_2``).
   * - ``tip_offset2``
     - ``0.0589379 -0.0807857 -0.011``
     - Position of P in the ``arm_r2`` frame (origin of
       ``closing_tip_1``).
   * - ``baumgarte``
     - ``0.2``
     - Fraction of the closure error removed per step, in (0, 1].
   * - ``max_force``
     - ``100``
     - Maximum norm of the force at P, in N. Only reached at start-up when
       the loop is open.
   * - ``initialize_with_mgd``
     - ``true``
     - Close the loop with the direct geometric model at the first step.
   * - ``a_x``, ``a_z``, ``b_x``, ``b_z``, ``l1``, ``l2``, ``phi1``,
       ``phi2``, ``psi1``, ``psi2``
     - see the xacro
     - Geometry of the direct geometric model, only used at start-up.
       ``a_z`` and ``b_z`` hold the **y** coordinates: the names are
       historical.

.. note::

   The tip offsets are given explicitly because Gazebo drops the massless
   ``closing_tip_*`` links when it converts the URDF: they become SDF frames
   that the plugin cannot look up. Update the offsets if the arms change.

Behaviour
---------

In a headless test with the five-bar standing vertically under gravity
(1 ms step, 5 s):

* with the start-up assembly, the closure error stays below 5 µm while the
  mechanism swings, with forces at P of a few hundredths of a newton;
* without it, starting 7 cm open, the force is clamped at ``max_force`` for a
  few steps, the loop closes, and the error then stays below 7 µm.

The plugin logs the closure error, the force and P\ :sub:`1` once per
simulated second at the debug level (``gz sim -v 4``). It warns when the force
is clamped.
