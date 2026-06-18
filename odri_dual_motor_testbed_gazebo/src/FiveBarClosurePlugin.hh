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

namespace odri_gz
{

class FiveBarClosurePluginPrivate;

/// \brief Gz-Sim System plugin that enforces the kinematic loop closure of a
/// five-bar parallel robot using a penalty-based (spring-damper) method.
///
/// The URDF→SDF conversion demotes zero-inertia dummy links (closing_ee_1,
/// closing_ee_2) to SDF frames that have no ECM entity.  The plugin therefore
/// references the real parent links (l2r, l2l) directly by name and adds a
/// constant offset (taken from the URDF fixed-joint xyz) to reach the actual
/// constraint point.
///
/// Forces are applied to a separate set of "force links" (rotor_asm,
/// rotor_asm_2, mass ~0.03 kg) rather than to l2r/l2l (mass 1e-9 kg),
/// which would cause ODE divergence.  The local offset to the force link is
/// recomputed each step because the elbow revolute joint between the force
/// link and the position link rotates that vector.
///
/// SDF parameters (all optional, shown with defaults):
///   <pos_link1>l2r</pos_link1>      - Link whose WorldPose anchors chain 1
///   <pos_link2>l2l</pos_link2>      - Link whose WorldPose anchors chain 2
///   <offset1>0 -0.125 -0.002</offset1>   - Offset in pos_link1 frame [m]
///   <offset2>0 -0.125 -0.0015</offset2>  - Offset in pos_link2 frame [m]
///   <parent_link1>rotor_asm</parent_link1>   - Force application link 1
///   <parent_link2>rotor_asm_2</parent_link2> - Force application link 2
///   <kp>50000.0</kp>                - Position error gain  [N/m]
///   <kd>500.0</kd>                  - Velocity error gain  [N·s/m]
///   <max_force>20.0</max_force>     - Per-step force clamp [N]
class FiveBarClosurePlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  FiveBarClosurePlugin();
  ~FiveBarClosurePlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  std::unique_ptr<FiveBarClosurePluginPrivate> dataPtr;
};

}  // namespace odri_gz

#endif  // FIVE_BAR_CLOSURE_PLUGIN_HH_
