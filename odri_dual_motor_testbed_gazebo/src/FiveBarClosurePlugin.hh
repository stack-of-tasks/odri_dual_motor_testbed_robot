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

#include <gz/sim/System.hh>
#include <memory>

namespace odri_gz {

class FiveBarClosurePluginPrivate;

/// \brief Gz-Sim System plugin that closes the kinematic loop of a five-bar
/// parallel robot with the force transmitted through the closure point P.
///
/// The model is loaded as two open chains base -(motor)-> crank -(elbow)->
/// coupler. The two couplers are linked at P by a planar pin constraint
/// c(q) = P1(q) - P2(q) = 0 (in-plane components only, the plane being normal
/// to the motor axes). The passive joints are never written: each PreUpdate
/// step the plugin computes the Lagrange multiplier lambda (the in-plane
/// force transmitted at P) and applies +lambda at P1 on coupler 1 and -lambda
/// at P2 on coupler 2, so that P emerges from the simulated dynamics.
///
/// lambda solves J M^-1 J^T lambda = -beta c/dt^2 - J qd/dt - J a_free, i.e.
/// after the step the constraint velocity is J qd+ = -beta c/dt (velocity
/// level Baumgarte stabilisation). M is the joint-space mass matrix of each
/// chain built from the link inertials in the ECM, J the point Jacobians of
/// P1/P2, and a_free the unconstrained joint acceleration, estimated from the
/// previous step (measured acceleration minus the part due to lambda), which
/// accounts for motor torques, damping, Coriolis terms and contacts without
/// modelling them.
///
/// At start-up the passive joints are assembled once on the closed-loop
/// manifold with the exact direct geometric model (five_bar_mgd_spec.md),
/// unless <initialize_with_mgd> is false.
///
/// SDF parameters (all optional, shown with defaults):
///   <motor_joint1>motorL</motor_joint1>  - Actuated joint, chain 1
///   <motor_joint2>motorR</motor_joint2>  - Actuated joint, chain 2
///   <elbow_joint1>elbowL</elbow_joint1>  - Passive joint, chain 1
///   <elbow_joint2>elbowR</elbow_joint2>  - Passive joint, chain 2
///   <tip_offset1>0.1 0 0</tip_offset1>   - P in the child link frame of
///                                          elbow_joint1
///   <tip_offset2>0.0589379 -0.0807857 -0.011</tip_offset2>
///                                        - P in the child link frame of
///                                          elbow_joint2
///   <baumgarte>0.2</baumgarte>    - Fraction of the closure error removed
///                                   per step (0..1]
///   <max_force>100</max_force>    - Safety clamp on |lambda| [N]
///   <initialize_with_mgd>true</initialize_with_mgd>
///   Geometry of the initial assembly (see five_bar_mgd_spec.md):
///   <a_x>-0.065</a_x> <a_z>0</a_z>  - motor_joint1 axis position (in plane)
///   <b_x>0.065</b_x> <b_z>0</b_z>   - motor_joint2 axis position (in plane)
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
