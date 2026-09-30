Passive joint model
===================

``five_bar_passive_joints.hpp`` is a header-only implementation of the direct
geometric model of the five-bar. It computes the passive joint positions and
velocities from the motor positions and velocities. It uses the same model as
the ``FiveBarClosurePlugin`` of ``odri_dual_motor_testbed_gazebo``; the full
derivation is in ``five_bar_mgd_spec.md`` at the root of the repository.

All coordinates are in the (x, y) plane of ``case``.

Positions
---------

The elbows are at

.. math::

   E_L = A + L_1 \begin{pmatrix} \cos(\varphi_1 - \theta_1) \\ \sin(\varphi_1 - \theta_1) \end{pmatrix},
   \qquad
   E_R = B + L_1 \begin{pmatrix} \cos(\varphi_2 - \theta_2) \\ \sin(\varphi_2 - \theta_2) \end{pmatrix}

The closure point P is the intersection of the two circles of radius
L\ :sub:`2` centred on E\ :sub:`L` and E\ :sub:`R`. With
d = ‖E\ :sub:`R` − E\ :sub:`L`‖, u = (E\ :sub:`R` − E\ :sub:`L`) / d and
perp(u) = (−u\ :sub:`y`, u\ :sub:`x`):

.. math::

   P = \frac{E_L + E_R}{2} + \sqrt{L_2^2 - \frac{d^2}{4}}\; \mathrm{perp}(u)

This branch is the one of the assembled mechanism. The passive angles are

.. math::

   \beta_i = \operatorname{atan2}(P - E_i) - \psi_i + \theta_i

No assembly exists when d > 2 L\ :sub:`2` or when the elbows coincide:
``ComputePassiveJoints`` then returns ``kOutOfWorkspace`` and leaves its
output unchanged.

Velocities
----------

Differentiating the two closure constraints
r\ :sub:`i` · (Ṗ − Ė\ :sub:`i`) = 0, with r\ :sub:`i` = P − E\ :sub:`i`,
gives a 2×2 linear system in Ṗ. Then

.. math::

   \dot\beta_i = \frac{r_i \times (\dot P - \dot E_i)}{L_2^2} + \dot\theta_i

When that system is singular, ``ComputePassiveJoints`` returns
``kSingularVelocity``: the positions are valid and the velocities are 0.

Parameters
----------

The parameters are held by ``FiveBarGeometry``. Its defaults are those of the
``fivebar_2dof`` model, listed in :doc:`five_bar_system`. At θ = 0 (encoder
zero) both cranks point along +y of ``case`` and the model gives
β\ :sub:`1` ≈ 0.8351 and β\ :sub:`2` ≈ −0.4169.

Tests
-----

``test/test_five_bar_passive_joints.cpp`` checks:

* ``EncoderZero``: the passive angles at θ = 0;
* ``VelocityMatchesFiniteDifference``: the analytical velocities against a
  finite difference of the positions;
* ``OutOfWorkspace``: the status returned when the loop cannot be closed.
