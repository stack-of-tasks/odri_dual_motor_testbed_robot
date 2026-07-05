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

#include <algorithm>
#include <cmath>
#include <string>

#include <gz/math/Vector2.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Element.hh>

namespace odri_gz {

namespace {

constexpr double kPi = 3.14159265358979323846;

// perp(u) = (-u.Y(), u.X()) -- the branch verified against the real
// assembly (see five_bar_mgd_spec.md, step 2).
gz::math::Vector2d Perp(const gz::math::Vector2d& _u) {
  return {-_u.Y(), _u.X()};
}

}  // namespace

// ---------------------------------------------------------------------------
class FiveBarClosurePluginPrivate {
 public:
  gz::sim::Model model{gz::sim::kNullEntity};

  // Actuated joints (inputs, theta1/theta2) and passive joints (outputs,
  // beta1/beta2) driven by the direct geometric model.
  std::string motorJointName1{"motorL"};
  std::string motorJointName2{"motorR"};
  std::string elbowJointName1{"elbowL"};
  std::string elbowJointName2{"elbowR"};

  // Geometry parameters, see five_bar_mgd_spec.md section 2.
  gz::math::Vector2d a{-0.065, 0.0};
  gz::math::Vector2d b{0.065, 0.0};
  double l1{0.06};
  double l2{0.125};
  double phi1{kPi};
  double phi2{1.9504681943696776};
  double psi1{1.1061036468685321};
  double psi2{2.6769066384729094};

  gz::sim::Joint motorJoint1;
  gz::sim::Joint motorJoint2;
  gz::sim::Joint elbowJoint1;
  gz::sim::Joint elbowJoint2;

  bool initialized{false};
  // Latches so out-of-workspace / degenerate-configuration warnings are
  // logged once on transition rather than every simulation step.
  bool outOfWorkspace{false};
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

  if (_sdf->HasElement("motor_joint1"))
    this->dataPtr->motorJointName1 = _sdf->Get<std::string>("motor_joint1");
  if (_sdf->HasElement("motor_joint2"))
    this->dataPtr->motorJointName2 = _sdf->Get<std::string>("motor_joint2");
  if (_sdf->HasElement("elbow_joint1"))
    this->dataPtr->elbowJointName1 = _sdf->Get<std::string>("elbow_joint1");
  if (_sdf->HasElement("elbow_joint2"))
    this->dataPtr->elbowJointName2 = _sdf->Get<std::string>("elbow_joint2");

  double aX = this->dataPtr->a.X();
  double aZ = this->dataPtr->a.Y();
  double bX = this->dataPtr->b.X();
  double bZ = this->dataPtr->b.Y();
  if (_sdf->HasElement("a_x")) aX = _sdf->Get<double>("a_x");
  if (_sdf->HasElement("a_z")) aZ = _sdf->Get<double>("a_z");
  if (_sdf->HasElement("b_x")) bX = _sdf->Get<double>("b_x");
  if (_sdf->HasElement("b_z")) bZ = _sdf->Get<double>("b_z");
  this->dataPtr->a = {aX, aZ};
  this->dataPtr->b = {bX, bZ};

  if (_sdf->HasElement("l1")) this->dataPtr->l1 = _sdf->Get<double>("l1");
  if (_sdf->HasElement("l2")) this->dataPtr->l2 = _sdf->Get<double>("l2");
  if (_sdf->HasElement("phi1")) this->dataPtr->phi1 = _sdf->Get<double>("phi1");
  if (_sdf->HasElement("phi2")) this->dataPtr->phi2 = _sdf->Get<double>("phi2");
  if (_sdf->HasElement("psi1")) this->dataPtr->psi1 = _sdf->Get<double>("psi1");
  if (_sdf->HasElement("psi2")) this->dataPtr->psi2 = _sdf->Get<double>("psi2");

  gzmsg << "[FiveBarClosurePlugin] Model='" << this->dataPtr->model.Name(_ecm)
        << "'"
        << "  motor_joint1=" << this->dataPtr->motorJointName1
        << "  motor_joint2=" << this->dataPtr->motorJointName2
        << "  elbow_joint1=" << this->dataPtr->elbowJointName1
        << "  elbow_joint2=" << this->dataPtr->elbowJointName2
        << "  a=" << this->dataPtr->a << "  b=" << this->dataPtr->b
        << "  l1=" << this->dataPtr->l1 << "  l2=" << this->dataPtr->l2
        << "  phi1=" << this->dataPtr->phi1 << "  phi2=" << this->dataPtr->phi2
        << "  psi1=" << this->dataPtr->psi1 << "  psi2=" << this->dataPtr->psi2
        << "\n";
}

// ---------------------------------------------------------------------------
static bool FindJoint(const gz::sim::Model& _model,
                      gz::sim::EntityComponentManager& _ecm,
                      const std::string& _name, gz::sim::Joint& _joint,
                      bool _enablePositionCheck) {
  gz::sim::Entity entity = _model.JointByName(_ecm, _name);
  if (entity == gz::sim::kNullEntity) {
    gzmsg << "[FiveBarClosurePlugin] joint '" << _name
          << "' not yet in ECM, retrying.\n";
    return false;
  }
  _joint = gz::sim::Joint(entity);
  if (_enablePositionCheck) _joint.EnablePositionCheck(_ecm);
  return true;
}

// ---------------------------------------------------------------------------
void FiveBarClosurePlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                     gz::sim::EntityComponentManager& _ecm) {
  if (_info.paused) return;

  auto& d = *this->dataPtr;

  if (!d.initialized) {
    bool ok = FindJoint(d.model, _ecm, d.motorJointName1, d.motorJoint1, true);
    ok = FindJoint(d.model, _ecm, d.motorJointName2, d.motorJoint2, true) && ok;
    ok =
        FindJoint(d.model, _ecm, d.elbowJointName1, d.elbowJoint1, false) && ok;
    ok =
        FindJoint(d.model, _ecm, d.elbowJointName2, d.elbowJoint2, false) && ok;
    if (!ok) return;
    d.initialized = true;
    gzmsg << "[FiveBarClosurePlugin] Exact loop-closure enforcement active.\n";
  }

  // Step 0 -- read actuated joint positions (theta1, theta2).
  auto pos1 = d.motorJoint1.Position(_ecm);
  auto pos2 = d.motorJoint2.Position(_ecm);
  if (!pos1 || pos1->empty() || !pos2 || pos2->empty()) return;
  const double theta1 = (*pos1)[0];
  const double theta2 = (*pos2)[0];

  // Step 1 -- crank tip positions (direct).
  const gz::math::Vector2d eL{d.a.X() + d.l1 * std::cos(d.phi1 - theta1),
                              d.a.Y() + d.l1 * std::sin(d.phi1 - theta1)};
  const gz::math::Vector2d eR{d.b.X() + d.l1 * std::cos(d.phi2 - theta2),
                              d.b.Y() + d.l1 * std::sin(d.phi2 - theta2)};

  // Step 2 -- P by intersection of two circles of radius L2.
  const gz::math::Vector2d eVec = eR - eL;
  double dEE = eVec.Length();
  if (dEE < 1e-9) {
    gzerr << "[FiveBarClosurePlugin] Degenerate configuration (crank tips "
             "coincide); skipping this step.\n";
    return;
  }

  const double dEEMax = 2.0 * d.l2;
  if (dEE > dEEMax) {
    if (!d.outOfWorkspace) {
      gzerr << "[FiveBarClosurePlugin] d_EE=" << dEE
            << " exceeds 2*L2=" << dEEMax
            << "; configuration outside workspace, clamping.\n";
      d.outOfWorkspace = true;
    }
    dEE = dEEMax;
  } else if (d.outOfWorkspace) {
    gzmsg << "[FiveBarClosurePlugin] Back inside workspace.\n";
    d.outOfWorkspace = false;
  }

  const gz::math::Vector2d u = eVec / dEE;
  const gz::math::Vector2d m = (eL + eR) / 2.0;
  const double h =
      std::sqrt(std::max(0.0, d.l2 * d.l2 - (dEE / 2.0) * (dEE / 2.0)));
  const gz::math::Vector2d p = m + h * Perp(u);

  // Step 3 -- elbow angles.
  const gz::math::Vector2d pMinusEL = p - eL;
  const gz::math::Vector2d pMinusER = p - eR;
  const double beta1 = std::atan2(pMinusEL.Y(), pMinusEL.X()) - d.psi1 + theta1;
  const double beta2 = std::atan2(pMinusER.Y(), pMinusER.X()) - d.psi2 + theta2;

  d.elbowJoint1.ResetPosition(_ecm, {beta1});
  d.elbowJoint2.ResetPosition(_ecm, {beta2});
}

}  // namespace odri_gz

// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(odri_gz::FiveBarClosurePlugin, gz::sim::System,
              odri_gz::FiveBarClosurePlugin::ISystemConfigure,
              odri_gz::FiveBarClosurePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(odri_gz::FiveBarClosurePlugin,
                    "odri_gz::FiveBarClosurePlugin")
