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

#include "FiveBarClosurePlugin.hh"

#include <chrono>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Element.hh>

namespace odri_gz {

// ---------------------------------------------------------------------------
// Data for one constraint endpoint.
//
// posLink  — the actual link in the ECM whose WorldPose is used to compute
//            the constraint point's world position.  For the five-bar robot
//            this is l2r / l2l (found by Model::LinkByName).  The SDF frames
//            closing_ee_1 / closing_ee_2 do NOT exist as ECM entities after
//            URDF→SDF conversion, so they cannot be used here.
//
// offset   — constant translation from posLink's origin to the constraint
//            point, expressed in the posLink frame.  Taken directly from the
//            URDF fixed-joint <origin xyz=…> and passed via SDF parameter.
//
// forceLink — link to which the penalty force is applied.  Must have
//             non-negligible mass to avoid ODE divergence (rotor_asm /
//             rotor_asm_2, ~0.03 kg each).  l2r / l2l have mass 1e-9 kg;
//             a 20 N force gives a ≈ 2×10¹⁰ m/s² — instant ODE overflow.
//
// The local offset from forceLink's origin to the constraint point changes
// every step (elbow revolute joint between forceLink and posLink), so it is
// recomputed from WorldPoses each PreUpdate call.
// ---------------------------------------------------------------------------
struct ConstraintPoint {
  gz::sim::Entity posLinkEntity{gz::sim::kNullEntity};
  gz::sim::Link posLink{gz::sim::kNullEntity};
  gz::math::Vector3d offset;  // constant, in posLink frame

  gz::sim::Entity forceLinkEntity{gz::sim::kNullEntity};
  gz::sim::Link forceLink{gz::sim::kNullEntity};

  bool ready{false};
};

class FiveBarClosurePluginPrivate {
 public:
  gz::sim::Model model{gz::sim::kNullEntity};

  // Position link names (real ECM links, children of revolute joints).
  std::string posLinkName1{"l2r"};
  std::string posLinkName2{"l2l"};

  // Fixed offsets in posLink frame from URDF closing_ee_*_frame joint xyz.
  gz::math::Vector3d offset1{0, -0.125, -0.002};
  gz::math::Vector3d offset2{0, -0.125, -0.0015};

  // Massive links used for force application.
  std::string forceLinkName1{"rotor_asm"};
  std::string forceLinkName2{"rotor_asm_2"};

  double kp{50000.0};
  double kd{500.0};
  // Force clamp [N]: prevents ODE AABB overflow when the initial configuration
  // violates loop closure (large initial error → explosive forces on ~0.03 kg).
  double maxForce{20.0};

  ConstraintPoint cp1;
  ConstraintPoint cp2;

  gz::math::Vector3d prevPosError{gz::math::Vector3d::Zero};
  bool firstUpdate{true};
  bool initialized{false};
};

// ---------------------------------------------------------------------------
FiveBarClosurePlugin::FiveBarClosurePlugin()
    : dataPtr(std::make_unique<FiveBarClosurePluginPrivate>()) {}

FiveBarClosurePlugin::~FiveBarClosurePlugin() = default;

// ---------------------------------------------------------------------------
void FiveBarClosurePlugin::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& /*_eventMgr*/) {
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "[FiveBarClosurePlugin] Plugin must be attached to a model.\n";
    return;
  }

  if (_sdf->HasElement("pos_link1"))
    this->dataPtr->posLinkName1 = _sdf->Get<std::string>("pos_link1");
  if (_sdf->HasElement("pos_link2"))
    this->dataPtr->posLinkName2 = _sdf->Get<std::string>("pos_link2");
  if (_sdf->HasElement("offset1"))
    this->dataPtr->offset1 = _sdf->Get<gz::math::Vector3d>("offset1");
  if (_sdf->HasElement("offset2"))
    this->dataPtr->offset2 = _sdf->Get<gz::math::Vector3d>("offset2");
  if (_sdf->HasElement("parent_link1"))
    this->dataPtr->forceLinkName1 = _sdf->Get<std::string>("parent_link1");
  if (_sdf->HasElement("parent_link2"))
    this->dataPtr->forceLinkName2 = _sdf->Get<std::string>("parent_link2");
  if (_sdf->HasElement("kp")) this->dataPtr->kp = _sdf->Get<double>("kp");
  if (_sdf->HasElement("kd")) this->dataPtr->kd = _sdf->Get<double>("kd");
  if (_sdf->HasElement("max_force"))
    this->dataPtr->maxForce = _sdf->Get<double>("max_force");

  gzmsg << "[FiveBarClosurePlugin] Model='" << this->dataPtr->model.Name(_ecm)
        << "'"
        << "  pos_link1=" << this->dataPtr->posLinkName1
        << "  offset1=" << this->dataPtr->offset1
        << "  pos_link2=" << this->dataPtr->posLinkName2
        << "  offset2=" << this->dataPtr->offset2
        << "  parent_link1=" << this->dataPtr->forceLinkName1
        << "  parent_link2=" << this->dataPtr->forceLinkName2
        << "  kp=" << this->dataPtr->kp << "  kd=" << this->dataPtr->kd
        << "  max_force=" << this->dataPtr->maxForce << "\n";
}

// ---------------------------------------------------------------------------
static bool initConstraintPoint(ConstraintPoint& cp,
                                const std::string& posLinkName,
                                const gz::math::Vector3d& offset,
                                const std::string& forceLinkName,
                                const gz::sim::Model& model,
                                gz::sim::EntityComponentManager& ecm) {
  if (cp.ready) return true;

  cp.posLinkEntity = model.LinkByName(ecm, posLinkName);
  if (cp.posLinkEntity == gz::sim::kNullEntity) {
    gzmsg << "[FiveBarClosurePlugin] pos_link '" << posLinkName
          << "' not yet in ECM, retrying.\n";
    return false;
  }
  cp.posLink = gz::sim::Link(cp.posLinkEntity);
  cp.offset = offset;
  cp.posLink.EnableVelocityChecks(ecm);

  cp.forceLinkEntity = model.LinkByName(ecm, forceLinkName);
  if (cp.forceLinkEntity == gz::sim::kNullEntity) {
    gzwarn << "[FiveBarClosurePlugin] force_link '" << forceLinkName
           << "' not found; falling back to pos_link.\n";
    cp.forceLinkEntity = cp.posLinkEntity;
  }
  cp.forceLink = gz::sim::Link(cp.forceLinkEntity);

  auto n1 = ecm.Component<gz::sim::components::Name>(cp.posLinkEntity);
  auto n2 = ecm.Component<gz::sim::components::Name>(cp.forceLinkEntity);
  gzmsg << "[FiveBarClosurePlugin] pos_link='" << (n1 ? n1->Data() : "?")
        << "'  offset=" << offset << "  force_link='" << (n2 ? n2->Data() : "?")
        << "'.\n";

  cp.ready = true;
  return true;
}

// ---------------------------------------------------------------------------
void FiveBarClosurePlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                     gz::sim::EntityComponentManager& _ecm) {
  if (_info.paused) return;

  if (!this->dataPtr->initialized) {
    bool ok1 = initConstraintPoint(
        this->dataPtr->cp1, this->dataPtr->posLinkName1, this->dataPtr->offset1,
        this->dataPtr->forceLinkName1, this->dataPtr->model, _ecm);
    bool ok2 = initConstraintPoint(
        this->dataPtr->cp2, this->dataPtr->posLinkName2, this->dataPtr->offset2,
        this->dataPtr->forceLinkName2, this->dataPtr->model, _ecm);
    if (!ok1 || !ok2) return;
    this->dataPtr->initialized = true;
    gzmsg << "[FiveBarClosurePlugin] Loop-closure enforcement active.\n";
  }

  auto& cp1 = this->dataPtr->cp1;
  auto& cp2 = this->dataPtr->cp2;

  // World positions of the two constraint points:
  //   p_world = R_world_posLink * offset + p_world_posLink
  auto optPose1 = cp1.posLink.WorldPose(_ecm);
  auto optPose2 = cp2.posLink.WorldPose(_ecm);
  if (!optPose1 || !optPose2) return;

  gz::math::Vector3d pos1 = optPose1->CoordPositionAdd(cp1.offset);
  gz::math::Vector3d pos2 = optPose2->CoordPositionAdd(cp2.offset);

  // Position error and finite-difference velocity error.
  gz::math::Vector3d posError = pos1 - pos2;
  double dtSec = std::chrono::duration<double>(_info.dt).count();
  gz::math::Vector3d velError{gz::math::Vector3d::Zero};
  if (!this->dataPtr->firstUpdate && dtSec > 1e-9) {
    velError = (posError - this->dataPtr->prevPosError) / dtSec;
  }
  this->dataPtr->prevPosError = posError;
  this->dataPtr->firstUpdate = false;

  // Penalty spring-damper force.
  gz::math::Vector3d force =
      -(this->dataPtr->kp * posError + this->dataPtr->kd * velError);

  // Clamp magnitude to prevent ODE AABB overflow on startup.
  double fLen = force.Length();
  if (fLen > this->dataPtr->maxForce && fLen > 1e-9) {
    force *= this->dataPtr->maxForce / fLen;
  }

  // World poses of force-application links.
  auto optFL1 = cp1.forceLink.WorldPose(_ecm);
  auto optFL2 = cp2.forceLink.WorldPose(_ecm);
  if (!optFL1 || !optFL2) return;

  // Local offset from forceLink origin to constraint point, in forceLink frame.
  // Recomputed every step because the elbow joint rotates this vector.
  gz::math::Vector3d off1 =
      optFL1->Rot().Inverse().RotateVector(pos1 - optFL1->Pos());
  gz::math::Vector3d off2 =
      optFL2->Rot().Inverse().RotateVector(pos2 - optFL2->Pos());

  cp1.forceLink.AddWorldForce(_ecm, force, off1);
  cp2.forceLink.AddWorldForce(_ecm, -force, off2);
}

}  // namespace odri_gz

// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(odri_gz::FiveBarClosurePlugin, gz::sim::System,
              odri_gz::FiveBarClosurePlugin::ISystemConfigure,
              odri_gz::FiveBarClosurePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(odri_gz::FiveBarClosurePlugin,
                    "odri_gz::FiveBarClosurePlugin")
