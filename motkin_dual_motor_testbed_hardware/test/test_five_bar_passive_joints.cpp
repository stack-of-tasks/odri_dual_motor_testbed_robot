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

#include <gtest/gtest.h>

#include <cmath>
#include <utility>

#include "motkin_dual_motor_testbed_hardware/five_bar_passive_joints.hpp"

using motkin_dual_motor_testbed_hardware::ComputePassiveJoints;
using motkin_dual_motor_testbed_hardware::FiveBarGeometry;
using motkin_dual_motor_testbed_hardware::FiveBarStatus;
using motkin_dual_motor_testbed_hardware::PassiveJointState;

// Reference values obtained by closing the loop on the full homogeneous
// transforms of fivebar_2dof.urdf.xacro (see five_bar_mgd_spec.md).
TEST(FiveBarPassiveJoints, EncoderZero) {
  PassiveJointState s;
  ASSERT_EQ(ComputePassiveJoints(0.0, 0.0, 0.0, 0.0, FiveBarGeometry{}, s),
            FiveBarStatus::kOk);
  EXPECT_NEAR(s.beta1, 0.8351435511972323, 1e-9);
  EXPECT_NEAR(s.beta2, -0.41691126056285954, 1e-9);
  EXPECT_DOUBLE_EQ(s.beta1_dot, 0.0);
  EXPECT_DOUBLE_EQ(s.beta2_dot, 0.0);
}

TEST(FiveBarPassiveJoints, VelocityMatchesFiniteDifference) {
  const FiveBarGeometry g;
  const double eps = 1e-6;
  for (double t1 = -0.8; t1 <= 0.8; t1 += 0.2) {
    for (double t2 = -0.8; t2 <= 0.8; t2 += 0.2) {
      for (const auto& td :
           {std::pair{1.0, 0.0}, std::pair{0.0, 1.0}, std::pair{0.7, -1.3}}) {
        PassiveJointState s, sp, sm;
        ASSERT_EQ(ComputePassiveJoints(t1, t2, td.first, td.second, g, s),
                  FiveBarStatus::kOk);
        ComputePassiveJoints(t1 + eps * td.first, t2 + eps * td.second, 0, 0, g,
                             sp);
        ComputePassiveJoints(t1 - eps * td.first, t2 - eps * td.second, 0, 0, g,
                             sm);
        const double fd1 = std::remainder(sp.beta1 - sm.beta1, 2 * M_PI);
        const double fd2 = std::remainder(sp.beta2 - sm.beta2, 2 * M_PI);
        EXPECT_NEAR(s.beta1_dot, fd1 / (2 * eps), 1e-6);
        EXPECT_NEAR(s.beta2_dot, fd2 / (2 * eps), 1e-6);
      }
    }
  }
}

TEST(FiveBarPassiveJoints, OutOfWorkspace) {
  FiveBarGeometry g;
  g.l2 = 0.01;  // couplers far too short to close the loop
  PassiveJointState s;
  EXPECT_EQ(ComputePassiveJoints(0.0, 0.0, 0.0, 0.0, g, s),
            FiveBarStatus::kOutOfWorkspace);
}
