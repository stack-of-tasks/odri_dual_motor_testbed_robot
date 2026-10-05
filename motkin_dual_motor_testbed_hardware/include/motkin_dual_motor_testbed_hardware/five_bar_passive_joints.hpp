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
// Passive joint positions and velocities of the motkin_dual_motor_testbed
// five-bar, from the two actuated joint positions and velocities.
//
// Same exact direct geometric model as motkin_gz::FiveBarClosurePlugin (see
// five_bar_mgd_spec.md at the root of motkin_dual_motor_testbed_robot):
//
//   E_L = A + L1 (cos(phi1 - theta1), sin(phi1 - theta1))
//   E_R = B + L1 (cos(phi2 - theta2), sin(phi2 - theta2))
//   P   = intersection of the two circles (E_L, L2), (E_R, L2), +perp branch
//   beta_i = atan2(P - E_i) - psi_i + theta_i
//
// The velocities come from differentiating the loop-closure constraints
// r_i . (Pdot - Edot_i) = 0 with r_i = P - E_i, which gives Pdot, then
// beta_i_dot = (r_i x (Pdot - Edot_i)) / L2^2 + theta_i_dot.
//
// Coordinates are in the (x, y) plane of `case`.

#ifndef MOTKIN_DUAL_MOTOR_TESTBED_HARDWARE__FIVE_BAR_PASSIVE_JOINTS_HPP_
#define MOTKIN_DUAL_MOTOR_TESTBED_HARDWARE__FIVE_BAR_PASSIVE_JOINTS_HPP_

#include <cmath>

namespace motkin_dual_motor_testbed_hardware {

/// Constant geometric parameters, derived from fivebar_2dof.urdf.xacro.
/// Keep in sync with gazebo/gazebo_fivebar_2dof.urdf.xacro and
/// five_bar_mgd_spec.md section 2.
struct FiveBarGeometry {
  double a_x = -0.0421006;
  double a_y = 0.0302628;
  double b_x = 0.0578994;
  double b_y = 0.0302628;
  double l1 = 0.06;
  double l2 = 0.1;
  double phi1 = 1.5707963267948966;
  double phi2 = 1.5707963267948966;
  double psi1 = 0.21205399999927563;
  double psi2 = 2.511306362956145;
};

enum class FiveBarStatus {
  kOk,
  /// Positions are valid but the velocity map is singular; velocities are 0.
  kSingularVelocity,
  /// No assembly exists (d_EE > 2 L2 or coincident elbows); outputs untouched.
  kOutOfWorkspace,
};

struct PassiveJointState {
  double beta1 = 0.0;
  double beta2 = 0.0;
  double beta1_dot = 0.0;
  double beta2_dot = 0.0;
};

inline FiveBarStatus ComputePassiveJoints(double theta1, double theta2,
                                          double theta1_dot, double theta2_dot,
                                          const FiveBarGeometry& g,
                                          PassiveJointState& out) {
  const double a1 = g.phi1 - theta1;
  const double a2 = g.phi2 - theta2;
  const double el_x = g.a_x + g.l1 * std::cos(a1);
  const double el_y = g.a_y + g.l1 * std::sin(a1);
  const double er_x = g.b_x + g.l1 * std::cos(a2);
  const double er_y = g.b_y + g.l1 * std::sin(a2);

  const double dx = er_x - el_x;
  const double dy = er_y - el_y;
  const double d_ee = std::hypot(dx, dy);
  if (d_ee < 1e-9 || d_ee > 2.0 * g.l2) {
    return FiveBarStatus::kOutOfWorkspace;
  }
  const double u_x = dx / d_ee;
  const double u_y = dy / d_ee;
  const double h = std::sqrt(g.l2 * g.l2 - 0.25 * d_ee * d_ee);
  // perp(u) = (-u_y, u_x): the branch of the real assembly.
  const double p_x = 0.5 * (el_x + er_x) - h * u_y;
  const double p_y = 0.5 * (el_y + er_y) + h * u_x;

  const double rl_x = p_x - el_x;
  const double rl_y = p_y - el_y;
  const double rr_x = p_x - er_x;
  const double rr_y = p_y - er_y;
  out.beta1 = std::atan2(rl_y, rl_x) - g.psi1 + theta1;
  out.beta2 = std::atan2(rr_y, rr_x) - g.psi2 + theta2;

  // dE/dtheta = L1 (sin(a), -cos(a)).
  const double del_x = g.l1 * std::sin(a1) * theta1_dot;
  const double del_y = -g.l1 * std::cos(a1) * theta1_dot;
  const double der_x = g.l1 * std::sin(a2) * theta2_dot;
  const double der_y = -g.l1 * std::cos(a2) * theta2_dot;

  // [r_L; r_R] Pdot = [r_L . Edot_L; r_R . Edot_R]
  const double det = rl_x * rr_y - rl_y * rr_x;
  if (std::abs(det) < 1e-9) {
    out.beta1_dot = 0.0;
    out.beta2_dot = 0.0;
    return FiveBarStatus::kSingularVelocity;
  }
  const double c1 = rl_x * del_x + rl_y * del_y;
  const double c2 = rr_x * der_x + rr_y * der_y;
  const double pd_x = (rr_y * c1 - rl_y * c2) / det;
  const double pd_y = (-rr_x * c1 + rl_x * c2) / det;

  const double l2_sq = g.l2 * g.l2;
  out.beta1_dot =
      (rl_x * (pd_y - del_y) - rl_y * (pd_x - del_x)) / l2_sq + theta1_dot;
  out.beta2_dot =
      (rr_x * (pd_y - der_y) - rr_y * (pd_x - der_x)) / l2_sq + theta2_dot;
  return FiveBarStatus::kOk;
}

}  // namespace motkin_dual_motor_testbed_hardware

#endif  // MOTKIN_DUAL_MOTOR_TESTBED_HARDWARE__FIVE_BAR_PASSIVE_JOINTS_HPP_
