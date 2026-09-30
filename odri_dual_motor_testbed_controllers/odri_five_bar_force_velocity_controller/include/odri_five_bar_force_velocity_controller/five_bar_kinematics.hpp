// Copyright 2026 LAAS-CNRS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Analytical kinematics of the odri_dual_motor_testbed five-bar mechanism.
//
// The testbed (see
// odri_dual_motor_testbed_description/robots/fivebar_2dof.urdf.xacro and
// five_bar_mgd_spec.md at the root of odri_dual_motor_testbed_robot) is a
// planar five-bar linkage: two actuated revolute joints motor_1/motor_2
// fixed to `case` at points A and B, each driving a crank of length L1 down
// to an elbow (E_L, E_R), themselves connected to the common closing point P
// through two coupler links of length L2. Motion happens in the (x, y)
// plane of `case` (both motor axes point along its local -z).
//
// NOTE: the member names a_z/b_z below are historical. The previous testbed
// moved in the (x, z) plane of base_asm; this one moves in the (x, y) plane
// of `case`, so a_z/b_z now hold the *y* coordinates of A and B. The algebra
// is unchanged -- it only ever treats them as the second in-plane axis.
//
// This header reimplements the direct geometric model from
// five_bar_mgd_spec.md (closed form, no iterative solver) and adds its
// closed-form velocity Jacobian dP/dtheta, obtained by implicit
// differentiation of the two loop-closure constraints
//
//   |P - E_L(theta1)|^2 = L2^2
//   |P - E_R(theta2)|^2 = L2^2
//
// No Pinocchio (or any other rigid-body library) is used: every quantity
// below is a direct algebraic function of the two actuated joint angles
// and of the constant geometric parameters extracted from the URDF.

#ifndef ODRI_FIVE_BAR_FORCE_VELOCITY_CONTROLLER__FIVE_BAR_KINEMATICS_HPP_
#define ODRI_FIVE_BAR_FORCE_VELOCITY_CONTROLLER__FIVE_BAR_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace odri_five_bar_force_velocity_controller {

/// Constant geometric parameters of the five-bar, taken from the URDF.
/// Defaults are the numeric values documented in five_bar_mgd_spec.md,
/// extracted from the origin tags of the motor_1/motor_2/passive_1/passive_2/
/// closing_tip_*_frame joints in fivebar_2dof.urdf.xacro.
struct FiveBarGeometry {
  double a_x = -0.0421006;
  double a_z = 0.0302628;
  double b_x = 0.0578994;
  double b_z = 0.0302628;
  double l1 = 0.06;
  double l2 = 0.1;
  double phi1 = 1.5707963267948966;
  double phi2 = 1.5707963267948966;
};

using Vec2 = std::array<double, 2>;
/// Row-major 2x2 matrix: {{J11, J12}, {J21, J22}}.
using Mat2 = std::array<std::array<double, 2>, 2>;

class OutOfWorkspace : public std::runtime_error {
 public:
  explicit OutOfWorkspace(const std::string& what) : std::runtime_error(what) {}
};

class SingularConfiguration : public std::runtime_error {
 public:
  explicit SingularConfiguration(const std::string& what)
      : std::runtime_error(what) {}
};

/// Direct position of the two elbows E_L(theta1), E_R(theta2).
inline void ElbowPositions(double theta1, double theta2,
                           const FiveBarGeometry& geom, Vec2& e_l, Vec2& e_r) {
  const double a1 = geom.phi1 - theta1;
  const double a2 = geom.phi2 - theta2;
  e_l = {geom.a_x + geom.l1 * std::cos(a1), geom.a_z + geom.l1 * std::sin(a1)};
  e_r = {geom.b_x + geom.l1 * std::cos(a2), geom.b_z + geom.l1 * std::sin(a2)};
}

/// Direct geometric model: closing point P(theta1, theta2).
inline Vec2 ClosingPoint(double theta1, double theta2,
                         const FiveBarGeometry& geom) {
  Vec2 e_l, e_r;
  ElbowPositions(theta1, theta2, geom, e_l, e_r);

  const double dx = e_r[0] - e_l[0];
  const double dz = e_r[1] - e_l[1];
  const double d_ee = std::hypot(dx, dz);
  if (d_ee > 2.0 * geom.l2) {
    throw OutOfWorkspace("d_EE=" + std::to_string(d_ee) +
                         " m > 2*L2=" + std::to_string(2.0 * geom.l2) +
                         " m: (theta1, theta2) has no valid five-bar assembly");
  }
  const double u_x = dx / d_ee;
  const double u_z = dz / d_ee;
  const double m_x = (e_l[0] + e_r[0]) / 2.0;
  const double m_z = (e_l[1] + e_r[1]) / 2.0;
  const double h =
      std::sqrt(std::max(geom.l2 * geom.l2 - (d_ee / 2.0) * (d_ee / 2.0), 0.0));
  // perp(u) = (-u_z, u_x); branch matching the real assembly
  // (see five_bar_mgd_spec.md, step 2).
  return {m_x - h * u_z, m_z + h * u_x};
}

/// Analytical velocity Jacobian of the five-bar closing point.
///
/// Computes J such that (xdot, zdot)^T = J * (theta1dot, theta2dot)^T,
/// with (x, z) the coordinates of the closing point P in the (x, z) plane
/// of `case` (the z suffixes name the second in-plane axis).
///
/// Derivation (implicit differentiation of the loop-closure constraints):
/// let r_L = P - E_L, r_R = P - E_R (each of norm L2). Differentiating
/// |P - E_L(theta1)|^2 = L2^2 and |P - E_R(theta2)|^2 = L2^2 w.r.t. time
/// gives two linear equations in (xdot, zdot):
///
///   r_L . Pdot = r_L . (dE_L/dtheta1) * theta1dot
///   r_R . Pdot = r_R . (dE_R/dtheta2) * theta2dot
///
/// i.e. C * Pdot = diag(d1, d2) * thetadot, with
/// C = [[r_Lx, r_Lz], [r_Rx, r_Rz]] and d1 = r_L . dE_L/dtheta1,
/// d2 = r_R . dE_R/dtheta2. Solving the 2x2 system for Pdot gives
/// J = C^-1 * diag(d1, d2), expanded below in closed form.
inline Mat2 Jacobian(double theta1, double theta2, const FiveBarGeometry& geom,
                     double singular_tol = 1e-9) {
  Vec2 e_l, e_r;
  ElbowPositions(theta1, theta2, geom, e_l, e_r);
  const Vec2 p = ClosingPoint(theta1, theta2, geom);

  const double r_lx = p[0] - e_l[0];
  const double r_lz = p[1] - e_l[1];
  const double r_rx = p[0] - e_r[0];
  const double r_rz = p[1] - e_r[1];

  const double a1 = geom.phi1 - theta1;
  const double a2 = geom.phi2 - theta2;
  // dE_L/dtheta1 = L1*(sin(a1), -cos(a1)); dE_R/dtheta2 = L1*(sin(a2),
  // -cos(a2))
  const double de_l_x = geom.l1 * std::sin(a1);
  const double de_l_z = -geom.l1 * std::cos(a1);
  const double de_r_x = geom.l1 * std::sin(a2);
  const double de_r_z = -geom.l1 * std::cos(a2);

  const double d1 = r_lx * de_l_x + r_lz * de_l_z;
  const double d2 = r_rx * de_r_x + r_rz * de_r_z;

  const double det = r_lx * r_rz - r_lz * r_rx;
  if (std::abs(det) < singular_tol) {
    throw SingularConfiguration(
        "five-bar Jacobian is singular (det=" + std::to_string(det) +
        ") at theta1=" + std::to_string(theta1) +
        ", theta2=" + std::to_string(theta2));
  }

  Mat2 j;
  j[0][0] = r_rz * d1 / det;
  j[0][1] = -r_lz * d2 / det;
  j[1][0] = -r_rx * d1 / det;
  j[1][1] = r_lx * d2 / det;
  return j;
}

/// Analytical gravity-compensation feed-forward torque for motor_1/motor_2.
///
/// DISABLED BY DEFAULT for the MOTKINBENCH five-bar, and kept only for a
/// vertically-mounted variant. Two assumptions below stopped holding when
/// the description was regenerated from MOTKINBENCH/urdf/five_bar:
///
///  1. The mechanism plane is now the (x, y) plane of `case`, with both
///     motor axes vertical. Gravity is therefore perpendicular to the
///     plane and exerts no torque on motor_1/motor_2 at all, so the
///     expression below (which differentiates the second *in-plane*
///     coordinate) no longer models anything physical.
///  2. The coupler centres of mass no longer sit at the elbows: arm_l2's
///     <inertial> origin is (0.0487, 0, -0.002) and arm_r2's is
///     (0.0328, -0.0450, -0.0031), both in their own frames, so E_L/E_R
///     are no longer the coupler CoM positions either.
///
/// Re-deriving this for a vertical remount means redoing both: picking the
/// two in-plane axes that contain gravity, and carrying the coupler CoM
/// offsets through the loop closure (they now depend on both thetas).
///
/// The original derivation, valid for the previous vertical testbed, used
/// the potential energy V(theta1,theta2) = g * (m_left * E_Lz(theta1) +
/// m_right * E_Rz(theta2)), giving tau_g = dV/dtheta:
///
///   tau_g1 = m_left  * g * dE_Lz/dtheta1 = m_left  * g *
///   (-L1*cos(phi1-theta1)) tau_g2 = m_right * g * dE_Rz/dtheta2 = m_right * g
///   * (-L1*cos(phi2-theta2))
///
/// The crank's own mass was deliberately not compensated: its CoM sat
/// within ~0.25 mm of the motor axis (the BLDC rotor dominates and is
/// symmetric about its spin axis), around 1% of the coupler term.
inline Vec2 GravityTorque(double theta1, double theta2,
                          const FiveBarGeometry& geom, double coupler_mass_left,
                          double coupler_mass_right, double gravity = 9.81) {
  const double a1 = geom.phi1 - theta1;
  const double a2 = geom.phi2 - theta2;
  const double de_l_z = -geom.l1 * std::cos(a1);
  const double de_r_z = -geom.l1 * std::cos(a2);
  return {coupler_mass_left * gravity * de_l_z,
          coupler_mass_right * gravity * de_r_z};
}

/// qdot = J^T * f_c: transpose-Jacobian force-to-velocity mapping.
///
/// f_c = (fx, fy) is the contact force at the closing point P, expressed
/// in the (x, y) plane of `case`. Using J^T (rather than the inverse
/// Jacobian) is a deliberate simplification for this teaching example: it
/// avoids any matrix inversion / singularity handling in the force path
/// and always pushes the joints in the direction that is power-conjugate
/// to the applied Cartesian force.
inline Vec2 JointVelocityFromContactForce(double theta1, double theta2,
                                          double fx, double fy,
                                          const FiveBarGeometry& geom) {
  const Mat2 j = Jacobian(theta1, theta2, geom);
  return {j[0][0] * fx + j[1][0] * fy, j[0][1] * fx + j[1][1] * fy};
}

}  // namespace odri_five_bar_force_velocity_controller

#endif  // ODRI_FIVE_BAR_FORCE_VELOCITY_CONTROLLER__FIVE_BAR_KINEMATICS_HPP_
