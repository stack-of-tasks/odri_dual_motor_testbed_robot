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
// Sanity checks for the analytical five-bar Jacobian: compares it against
// a central finite-difference of the direct geometric model (ClosingPoint)
// over a handful of configurations spanning the workspace.

#include <gmock/gmock.h>

#include <cmath>
#include <utility>
#include <vector>

#include "motkin_five_bar_force_velocity_controller/five_bar_kinematics.hpp"

using motkin_five_bar_force_velocity_controller::ClosingPoint;
using motkin_five_bar_force_velocity_controller::ElbowPositions;
using motkin_five_bar_force_velocity_controller::FiveBarGeometry;
using motkin_five_bar_force_velocity_controller::GravityTorque;
using motkin_five_bar_force_velocity_controller::Jacobian;
using motkin_five_bar_force_velocity_controller::JointVelocityFromContactForce;
using motkin_five_bar_force_velocity_controller::Vec2;

namespace {

constexpr double kFdStep = 1e-6;
constexpr double kTol = 1e-4;

}  // namespace

TEST(FiveBarKinematics, JacobianMatchesFiniteDifference) {
  FiveBarGeometry geom;

  const std::vector<std::pair<double, double>> configs = {
      {0.0, 0.0}, {0.3, -0.2},  {-0.4, 0.5},
      {0.6, 0.6}, {-0.6, -0.6}, {0.1, -0.5},
  };

  for (const auto& [theta1, theta2] : configs) {
    const auto j = Jacobian(theta1, theta2, geom);

    const auto p_plus1 = ClosingPoint(theta1 + kFdStep, theta2, geom);
    const auto p_minus1 = ClosingPoint(theta1 - kFdStep, theta2, geom);
    const double dpx_dtheta1 = (p_plus1[0] - p_minus1[0]) / (2 * kFdStep);
    const double dpz_dtheta1 = (p_plus1[1] - p_minus1[1]) / (2 * kFdStep);

    const auto p_plus2 = ClosingPoint(theta1, theta2 + kFdStep, geom);
    const auto p_minus2 = ClosingPoint(theta1, theta2 - kFdStep, geom);
    const double dpx_dtheta2 = (p_plus2[0] - p_minus2[0]) / (2 * kFdStep);
    const double dpz_dtheta2 = (p_plus2[1] - p_minus2[1]) / (2 * kFdStep);

    // j = {{J11, J12}, {J21, J22}} with xdot = J11*theta1dot + J12*theta2dot
    // and zdot = J21*theta1dot + J22*theta2dot.
    EXPECT_NEAR(j[0][0], dpx_dtheta1, kTol);
    EXPECT_NEAR(j[0][1], dpx_dtheta2, kTol);
    EXPECT_NEAR(j[1][0], dpz_dtheta1, kTol);
    EXPECT_NEAR(j[1][1], dpz_dtheta2, kTol);
  }
}

TEST(FiveBarKinematics, ForceVelocityIsPowerConjugateToJacobianTranspose) {
  FiveBarGeometry geom;
  const double theta1 = 0.2;
  const double theta2 = -0.3;
  const double fx = 1.5;
  const double fy = -0.7;

  const auto j = Jacobian(theta1, theta2, geom);
  const auto qdot = JointVelocityFromContactForce(theta1, theta2, fx, fy, geom);

  EXPECT_NEAR(qdot[0], j[0][0] * fx + j[1][0] * fy, 1e-12);
  EXPECT_NEAR(qdot[1], j[0][1] * fx + j[1][1] * fy, 1e-12);
}

TEST(FiveBarKinematics, GravityTorqueMatchesFiniteDifferenceOfPotentialEnergy) {
  FiveBarGeometry geom;
  const double g = 9.81;
  const double m_left = 0.01;
  const double m_right = 0.01;

  auto potential_energy = [&](double theta1, double theta2) {
    Vec2 e_l, e_r;
    ElbowPositions(theta1, theta2, geom, e_l, e_r);
    return g * (m_left * e_l[1] + m_right * e_r[1]);
  };

  const std::vector<std::pair<double, double>> configs = {
      {0.0, 0.0}, {0.3, -0.2}, {-0.4, 0.5}, {0.6, 0.6}, {-0.6, -0.6},
  };

  for (const auto& [theta1, theta2] : configs) {
    const Vec2 tau_g = GravityTorque(theta1, theta2, geom, m_left, m_right, g);

    const double dv_dtheta1 = (potential_energy(theta1 + kFdStep, theta2) -
                               potential_energy(theta1 - kFdStep, theta2)) /
                              (2 * kFdStep);
    const double dv_dtheta2 = (potential_energy(theta1, theta2 + kFdStep) -
                               potential_energy(theta1, theta2 - kFdStep)) /
                              (2 * kFdStep);

    EXPECT_NEAR(tau_g[0], dv_dtheta1, kTol);
    EXPECT_NEAR(tau_g[1], dv_dtheta2, kTol);
  }
}
