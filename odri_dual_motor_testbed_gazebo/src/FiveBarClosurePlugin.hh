// Copyright 2024 LAAS-CNRS
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

#ifndef FIVE_BAR_CLOSURE_PLUGIN_HH_
#define FIVE_BAR_CLOSURE_PLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace odri_gz {

class FiveBarClosurePluginPrivate;

/// \brief Gz-Sim System plugin that enforces the kinematic loop closure of a
/// five-bar parallel robot using its exact direct geometric model (no
/// integration, no gains, no penalty force).
///
/// The five-bar mechanism has two actuated revolute joints (motorL, motorR)
/// and two passive revolute joints (elbowL, elbowR) whose positions are
/// entirely determined by the actuated joint positions through the
/// loop-closure constraint. Each PreUpdate step this plugin reads
/// theta1 = motorL and theta2 = motorR from the ECM, computes the two crank
/// tips E_L(theta1) / E_R(theta2), intersects the two length-L2 circles
/// centered on them to find the closure point P, derives beta1 / beta2 and
/// writes them to elbowL / elbowR via Joint::ResetPosition. See
/// five_bar_mgd_spec.md at the repository root for the full derivation and
/// the numerical values of the geometry parameters below.
///
/// SDF parameters (all optional, shown with defaults):
///   <motor_joint1>motorL</motor_joint1>  - Actuated joint providing theta1
///   <motor_joint2>motorR</motor_joint2>  - Actuated joint providing theta2
///   <elbow_joint1>elbowL</elbow_joint1>  - Passive joint receiving beta1
///   <elbow_joint2>elbowR</elbow_joint2>  - Passive joint receiving beta2
///   <a_x>-0.065</a_x> <a_z>0</a_z>  - motorL axis position in base_asm (x,z)
///   <b_x>0.065</b_x> <b_z>0</b_z>   - motorR axis position in base_asm (x,z)
///   <l1>0.06</l1>              - Crank length (motor -> elbow)
///   <l2>0.125</l2>             - Coupler length (elbow -> closure point)
///   <phi1>pi</phi1>                  - Left crank angular offset [rad]
///   <phi2>1.9504681943696776</phi2>  - Right crank angular offset [rad]
///   <psi1>1.1061036468685321</psi1>  - Left coupler angular offset [rad]
///   <psi2>2.6769066384729094</psi2>  - Right coupler angular offset [rad]
class FiveBarClosurePlugin : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate {
 public:
  FiveBarClosurePlugin();
  ~FiveBarClosurePlugin() override;

  void Configure(const gz::sim::Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 gz::sim::EntityComponentManager& _ecm,
                 gz::sim::EventManager& _eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo& _info,
                 gz::sim::EntityComponentManager& _ecm) override;

 private:
  std::unique_ptr<FiveBarClosurePluginPrivate> dataPtr;
};

}  // namespace odri_gz

#endif  // FIVE_BAR_CLOSURE_PLUGIN_HH_
