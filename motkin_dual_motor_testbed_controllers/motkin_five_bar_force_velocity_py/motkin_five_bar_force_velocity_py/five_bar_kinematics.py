# Copyright 2026 LAAS-CNRS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Analytical kinematics of the motkin_dual_motor_testbed five-bar mechanism.

The testbed (see
motkin_dual_motor_testbed_description/robots/fivebar_2dof.urdf.xacro and
five_bar_mgd_spec.md at the root of motkin_dual_motor_testbed_robot) is a
planar five-bar linkage: two actuated revolute joints ``motor_1``/``motor_2``
fixed to ``case`` at points A and B, each driving a crank of length L1
down to an elbow (E_L, E_R), themselves connected to the common closing
point P through two coupler links of length L2. Motion happens in the
(x, y) plane of the ``case`` link (both motor axes point along its local
-z).

NOTE: the field names ``a_z``/``b_z`` below are historical. The previous
testbed moved in the (x, z) plane of ``base_asm``; this one moves in the
(x, y) plane of ``case``, so ``a_z``/``b_z`` now hold the *y* coordinates
of A and B. The algebra is unchanged: it only ever treats them as the
second in-plane axis.

This module reimplements the direct geometric model from
``five_bar_mgd_spec.md`` (closed form, no iterative solver) and adds its
closed-form velocity Jacobian dP/dtheta, obtained by implicit
differentiation of the two loop-closure constraints::

    |P - E_L(theta1)|^2 = L2^2
    |P - E_R(theta2)|^2 = L2^2

No Pinocchio (or any other rigid-body library) is used: every quantity
below is a direct algebraic function of the two actuated joint angles and
of the constant geometric parameters extracted from the URDF.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FiveBarGeometry:
    """Constant geometric parameters of the five-bar, taken from the URDF.

    Defaults are the numeric values documented in five_bar_mgd_spec.md,
    extracted from the ``origin`` tags of the ``motor_1``/``motor_2``/
    ``passive_1``/``passive_2``/``closing_tip_*_frame`` joints in
    fivebar_2dof.urdf.xacro.
    """

    a_x: float = -0.0421006
    a_z: float = 0.0302628
    b_x: float = 0.0578994
    b_z: float = 0.0302628
    l1: float = 0.06
    l2: float = 0.1
    phi1: float = 1.5707963267948966
    phi2: float = 1.5707963267948966


class OutOfWorkspaceError(ValueError):
    """Raised when the requested (theta1, theta2) has no valid assembly."""


class SingularConfigurationError(ValueError):
    """Raised when the five-bar Jacobian is (numerically) singular."""


def elbow_positions(
    theta1: float, theta2: float, geom: FiveBarGeometry
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Direct position of the two elbows E_L(theta1), E_R(theta2)."""
    a1 = geom.phi1 - theta1
    a2 = geom.phi2 - theta2
    e_l = (geom.a_x + geom.l1 * math.cos(a1), geom.a_z + geom.l1 * math.sin(a1))
    e_r = (geom.b_x + geom.l1 * math.cos(a2), geom.b_z + geom.l1 * math.sin(a2))
    return e_l, e_r


def closing_point(
    theta1: float, theta2: float, geom: FiveBarGeometry
) -> tuple[float, float]:
    """Direct geometric model: closing point P(theta1, theta2)."""
    e_l, e_r = elbow_positions(theta1, theta2, geom)
    dx, dz = e_r[0] - e_l[0], e_r[1] - e_l[1]
    d_ee = math.hypot(dx, dz)
    if d_ee > 2.0 * geom.l2:
        raise OutOfWorkspaceError(
            f"d_EE={d_ee:.6f} m > 2*L2={2.0 * geom.l2:.6f} m: "
            "(theta1, theta2) has no valid five-bar assembly"
        )
    u_x, u_z = dx / d_ee, dz / d_ee
    m_x, m_z = (e_l[0] + e_r[0]) / 2.0, (e_l[1] + e_r[1]) / 2.0
    h = math.sqrt(max(geom.l2 * geom.l2 - (d_ee / 2.0) ** 2, 0.0))
    # perp(u) = (-u_z, u_x); this is the branch matching the real assembly
    # (see five_bar_mgd_spec.md, step 2).
    return (m_x - h * u_z, m_z + h * u_x)


def jacobian(
    theta1: float,
    theta2: float,
    geom: FiveBarGeometry,
    singular_tol: float = 1e-9,
):
    """Analytical velocity Jacobian of the five-bar closing point.

    Returns ``(J, P, (E_L, E_R))`` where ``J = ((J11, J12), (J21, J22))`` is
    such that

        (xdot, zdot)^T = J @ (theta1dot, theta2dot)^T

    with (x, z) the coordinates of the closing point P in the (x, y)
    plane of ``case``, and ``P = (Px, Pz)`` the current closing point (the
    ``z`` suffixes are historical names for the second in-plane axis).

    Derivation (implicit differentiation of the loop-closure constraints):
    let r_L = P - E_L, r_R = P - E_R (each of norm L2). Differentiating
    ``|P - E_L(theta1)|^2 = L2^2`` and ``|P - E_R(theta2)|^2 = L2^2`` w.r.t. time
    gives two linear equations in (xdot, zdot):

        r_L . Pdot = r_L . (dE_L/dtheta1) * theta1dot
        r_R . Pdot = r_R . (dE_R/dtheta2) * theta2dot

    i.e. C @ Pdot = diag(d1, d2) @ thetadot, with
    C = [[r_Lx, r_Lz], [r_Rx, r_Rz]] and
    d1 = r_L . dE_L/dtheta1, d2 = r_R . dE_R/dtheta2. Solving the 2x2
    system C @ Pdot = diag(d1, d2) @ thetadot for Pdot gives J = C^-1 @
    diag(d1, d2), expanded below in closed form.
    """
    e_l, e_r = elbow_positions(theta1, theta2, geom)
    p_x, p_z = closing_point(theta1, theta2, geom)

    r_lx, r_lz = p_x - e_l[0], p_z - e_l[1]
    r_rx, r_rz = p_x - e_r[0], p_z - e_r[1]

    a1 = geom.phi1 - theta1
    a2 = geom.phi2 - theta2
    # dE_L/dtheta1 = L1 * (sin(a1), -cos(a1)); dE_R/dtheta2 = L1 * (sin(a2), -cos(a2))
    de_l_x, de_l_z = geom.l1 * math.sin(a1), -geom.l1 * math.cos(a1)
    de_r_x, de_r_z = geom.l1 * math.sin(a2), -geom.l1 * math.cos(a2)

    d1 = r_lx * de_l_x + r_lz * de_l_z
    d2 = r_rx * de_r_x + r_rz * de_r_z

    det = r_lx * r_rz - r_lz * r_rx
    if abs(det) < singular_tol:
        raise SingularConfigurationError(
            f"five-bar Jacobian is singular (det={det:.3e}) at "
            f"theta1={theta1:.4f}, theta2={theta2:.4f}"
        )

    j11 = r_rz * d1 / det
    j12 = -r_lz * d2 / det
    j21 = -r_rx * d1 / det
    j22 = r_lx * d2 / det

    return ((j11, j12), (j21, j22)), (p_x, p_z), (e_l, e_r)


def gravity_torque(
    theta1: float,
    theta2: float,
    geom: FiveBarGeometry,
    coupler_mass_left: float,
    coupler_mass_right: float,
    gravity: float = 9.81,
) -> tuple[float, float]:
    """Analytical gravity-compensation feed-forward torque for motor_1/motor_2.

    DISABLED BY DEFAULT for the MOTKINBENCH five-bar, and kept only for a
    vertically-mounted variant. Two assumptions below stopped holding when
    the description was regenerated from MOTKINBENCH/urdf/five_bar:

    1. The mechanism plane is now the (x, y) plane of ``case``, with both
       motor axes vertical. Gravity is therefore perpendicular to the plane
       and exerts no torque on ``motor_1``/``motor_2`` at all, so the
       expression below (which differentiates the second *in-plane*
       coordinate) no longer models anything physical.
    2. The coupler centres of mass no longer sit at the elbows: ``arm_l2``'s
       ``<inertial>`` origin is (0.0487, 0, -0.002) and ``arm_r2``'s is
       (0.0328, -0.0450, -0.0031), both in their own frames, so E_L/E_R are
       no longer the coupler CoM positions either.

    Re-deriving this for a vertical remount means redoing both: picking the
    two in-plane axes that contain gravity, and carrying the coupler CoM
    offsets through the loop closure (they now depend on both thetas).

    The original derivation, valid for the previous vertical testbed, used
    the potential energy V(theta1, theta2) = g * (m_left * E_Lz(theta1) +
    m_right * E_Rz(theta2)), giving tau_g = dV/dtheta:

        tau_g1 = m_left  * g * dE_Lz/dtheta1 = m_left  * g * (-L1*cos(phi1-theta1))
        tau_g2 = m_right * g * dE_Rz/dtheta2 = m_right * g * (-L1*cos(phi2-theta2))

    The crank's own mass was deliberately not compensated: its CoM sat
    within ~0.25 mm of the motor axis (the BLDC rotor dominates and is
    symmetric about its spin axis), around 1% of the coupler term.
    """
    a1 = geom.phi1 - theta1
    a2 = geom.phi2 - theta2
    de_l_z = -geom.l1 * math.cos(a1)
    de_r_z = -geom.l1 * math.cos(a2)
    return (
        coupler_mass_left * gravity * de_l_z,
        coupler_mass_right * gravity * de_r_z,
    )


def joint_velocity_from_contact_force(
    theta1: float,
    theta2: float,
    fx: float,
    fy: float,
    geom: FiveBarGeometry,
) -> tuple[float, float]:
    """qdot = J^T @ f_c: transpose-Jacobian force-to-velocity mapping.

    f_c = (fx, fy) is the contact force at the closing point P, expressed
    in the (x, y) plane of `case`. Using J^T (rather than the inverse
    Jacobian) is a deliberate simplification for this teaching example: it
    avoids any matrix inversion / singularity handling in the force path
    and always pushes the joints in the direction that is power-conjugate
    to the applied Cartesian force.
    """
    (j11, j12), (j21, j22) = jacobian(theta1, theta2, geom)[0]
    qdot1 = j11 * fx + j21 * fy
    qdot2 = j12 * fx + j22 * fy
    return qdot1, qdot2
