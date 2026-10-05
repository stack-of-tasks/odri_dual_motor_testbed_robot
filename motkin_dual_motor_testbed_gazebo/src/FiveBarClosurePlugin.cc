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
#include <array>
#include <chrono>
#include <cmath>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Element.hh>
#include <string>

namespace motkin_gz {

namespace {

constexpr double kPi = 3.14159265358979323846;

// perp(u) = (-u.Y(), u.X()) -- the branch verified against the real
// assembly (see five_bar_mgd_spec.md, step 2).
gz::math::Vector2d Perp(const gz::math::Vector2d& _u) {
  return {-_u.Y(), _u.X()};
}

// Minimal 2x2 matrix, stored row-major; enough for the per-chain mass
// matrices and the 2x2 constraint-space operator.
struct Mat2 {
  double m00{0.0}, m01{0.0}, m10{0.0}, m11{0.0};

  static Mat2 FromColumns(const gz::math::Vector2d& _c0,
                          const gz::math::Vector2d& _c1) {
    return {_c0.X(), _c1.X(), _c0.Y(), _c1.Y()};
  }
  Mat2 Transposed() const { return {m00, m10, m01, m11}; }
  double Det() const { return m00 * m11 - m01 * m10; }
  Mat2 Inverse() const {
    const double det = this->Det();
    return {m11 / det, -m01 / det, -m10 / det, m00 / det};
  }
  Mat2 operator*(const Mat2& _o) const {
    return {m00 * _o.m00 + m01 * _o.m10, m00 * _o.m01 + m01 * _o.m11,
            m10 * _o.m00 + m11 * _o.m10, m10 * _o.m01 + m11 * _o.m11};
  }
  Mat2 operator+(const Mat2& _o) const {
    return {m00 + _o.m00, m01 + _o.m01, m10 + _o.m10, m11 + _o.m11};
  }
  gz::math::Vector2d operator*(const gz::math::Vector2d& _v) const {
    return {m00 * _v.X() + m01 * _v.Y(), m10 * _v.X() + m11 * _v.Y()};
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// One arm of the five-bar: base -(motor)-> crank -(elbow)-> coupler, whose
// tip carries the closure point P.
struct FiveBarChain {
  std::string motorJointName;
  std::string elbowJointName;
  // Closure point P expressed in the coupler link frame (the frames
  // closing_tip_* are demoted to SDF frames and are absent from the ECM).
  gz::math::Vector3d tipOffset;

  gz::sim::Joint motorJoint{gz::sim::kNullEntity};
  gz::sim::Joint elbowJoint{gz::sim::kNullEntity};
  gz::sim::Link crankLink{gz::sim::kNullEntity};    // child of motorJoint
  gz::sim::Link couplerLink{gz::sim::kNullEntity};  // child of elbowJoint

  // Per-step kinematic/dynamic quantities, all in world frame.
  gz::math::Vector3d p;                   // closure point on this arm
  std::array<gz::math::Vector3d, 2> jac;  // dp/dq for (motor, elbow)
  Mat2 mass;                              // 2x2 joint-space mass matrix
  gz::math::Vector2d qd;                  // (motor, elbow) velocities
};

// ---------------------------------------------------------------------------
class FiveBarClosurePluginPrivate {
 public:
  bool InitChain(FiveBarChain& _c, gz::sim::EntityComponentManager& _ecm);
  bool UpdateChain(FiveBarChain& _c,
                   const gz::sim::EntityComponentManager& _ecm);
  bool ComputeElbowAngles(double _theta1, double _theta2, double& _beta1,
                          double& _beta2);

  gz::sim::Model model{gz::sim::kNullEntity};

  FiveBarChain chain1{"motorL", "elbowL", {0.1, 0.0, 0.0}};
  FiveBarChain chain2{"motorR", "elbowR", {0.0589379, -0.0807857, -0.011}};

  // Constraint parameters.
  double baumgarte{0.2};   // fraction of the position error removed per step
  double maxForce{100.0};  // safety clamp on |lambda| [N]

  // Geometry parameters of the direct geometric model, only used to assemble
  // the loop once at start-up (see five_bar_mgd_spec.md section 2).
  bool initializeWithMgd{true};
  gz::math::Vector2d a{-0.065, 0.0};
  gz::math::Vector2d b{0.065, 0.0};
  double l1{0.06};
  double l2{0.125};
  double phi1{kPi};
  double phi2{1.9504681943696776};
  double psi1{1.1061036468685321};
  double psi2{2.6769066384729094};

  bool initialized{false};

  // Time-delay estimate of the unconstrained joint acceleration: the
  // velocities of the previous step and the part of the last acceleration
  // that was due to the constraint force we applied.
  bool havePrev{false};
  std::array<gz::math::Vector2d, 2> qdPrev;
  std::array<gz::math::Vector2d, 2> constraintAccPrev;

  // Latches so warnings are logged once on transition rather than every step.
  bool clamping{false};
  std::chrono::steady_clock::duration lastLog{0};
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
  auto& d = *this->dataPtr;
  d.model = gz::sim::Model(_entity);
  if (!d.model.Valid(_ecm)) {
    gzerr << "[FiveBarClosurePlugin] Plugin must be attached to a model.\n";
    return;
  }

  if (_sdf->HasElement("motor_joint1"))
    d.chain1.motorJointName = _sdf->Get<std::string>("motor_joint1");
  if (_sdf->HasElement("motor_joint2"))
    d.chain2.motorJointName = _sdf->Get<std::string>("motor_joint2");
  if (_sdf->HasElement("elbow_joint1"))
    d.chain1.elbowJointName = _sdf->Get<std::string>("elbow_joint1");
  if (_sdf->HasElement("elbow_joint2"))
    d.chain2.elbowJointName = _sdf->Get<std::string>("elbow_joint2");
  if (_sdf->HasElement("tip_offset1"))
    d.chain1.tipOffset = _sdf->Get<gz::math::Vector3d>("tip_offset1");
  if (_sdf->HasElement("tip_offset2"))
    d.chain2.tipOffset = _sdf->Get<gz::math::Vector3d>("tip_offset2");

  if (_sdf->HasElement("baumgarte"))
    d.baumgarte = _sdf->Get<double>("baumgarte");
  if (_sdf->HasElement("max_force"))
    d.maxForce = _sdf->Get<double>("max_force");

  if (_sdf->HasElement("initialize_with_mgd"))
    d.initializeWithMgd = _sdf->Get<bool>("initialize_with_mgd");
  double aX = d.a.X();
  double aZ = d.a.Y();
  double bX = d.b.X();
  double bZ = d.b.Y();
  if (_sdf->HasElement("a_x")) aX = _sdf->Get<double>("a_x");
  if (_sdf->HasElement("a_z")) aZ = _sdf->Get<double>("a_z");
  if (_sdf->HasElement("b_x")) bX = _sdf->Get<double>("b_x");
  if (_sdf->HasElement("b_z")) bZ = _sdf->Get<double>("b_z");
  d.a = {aX, aZ};
  d.b = {bX, bZ};

  if (_sdf->HasElement("l1")) d.l1 = _sdf->Get<double>("l1");
  if (_sdf->HasElement("l2")) d.l2 = _sdf->Get<double>("l2");
  if (_sdf->HasElement("phi1")) d.phi1 = _sdf->Get<double>("phi1");
  if (_sdf->HasElement("phi2")) d.phi2 = _sdf->Get<double>("phi2");
  if (_sdf->HasElement("psi1")) d.psi1 = _sdf->Get<double>("psi1");
  if (_sdf->HasElement("psi2")) d.psi2 = _sdf->Get<double>("psi2");

  gzmsg << "[FiveBarClosurePlugin] Model='" << d.model.Name(_ecm) << "'"
        << "  motor_joint1=" << d.chain1.motorJointName
        << "  elbow_joint1=" << d.chain1.elbowJointName
        << "  tip_offset1=" << d.chain1.tipOffset
        << "  motor_joint2=" << d.chain2.motorJointName
        << "  elbow_joint2=" << d.chain2.elbowJointName
        << "  tip_offset2=" << d.chain2.tipOffset
        << "  baumgarte=" << d.baumgarte << "  max_force=" << d.maxForce
        << "  initialize_with_mgd=" << d.initializeWithMgd << "\n";
}

// ---------------------------------------------------------------------------
static bool FindJoint(const gz::sim::Model& _model,
                      gz::sim::EntityComponentManager& _ecm,
                      const std::string& _name, gz::sim::Joint& _joint) {
  gz::sim::Entity entity = _model.JointByName(_ecm, _name);
  if (entity == gz::sim::kNullEntity) {
    gzmsg << "[FiveBarClosurePlugin] joint '" << _name
          << "' not yet in ECM, retrying.\n";
    return false;
  }
  _joint = gz::sim::Joint(entity);
  _joint.EnablePositionCheck(_ecm);
  _joint.EnableVelocityCheck(_ecm);
  return true;
}

// Child link of a joint, looked up by name in the model.
static bool FindChildLink(const gz::sim::Model& _model,
                          gz::sim::EntityComponentManager& _ecm,
                          const gz::sim::Joint& _joint, gz::sim::Link& _link) {
  auto name = _joint.ChildLinkName(_ecm);
  if (!name) return false;
  gz::sim::Entity entity = _model.LinkByName(_ecm, *name);
  if (entity == gz::sim::kNullEntity) return false;
  _link = gz::sim::Link(entity);
  // Link::AddWorldForce needs the WorldPose component, which the physics
  // system only keeps up to date if it exists.
  if (!_ecm.Component<gz::sim::components::WorldPose>(entity))
    _ecm.CreateComponent(entity, gz::sim::components::WorldPose());
  return true;
}

bool FiveBarClosurePluginPrivate::InitChain(
    FiveBarChain& _c, gz::sim::EntityComponentManager& _ecm) {
  if (!FindJoint(this->model, _ecm, _c.motorJointName, _c.motorJoint) ||
      !FindJoint(this->model, _ecm, _c.elbowJointName, _c.elbowJoint))
    return false;
  if (!FindChildLink(this->model, _ecm, _c.motorJoint, _c.crankLink) ||
      !FindChildLink(this->model, _ecm, _c.elbowJoint, _c.couplerLink)) {
    gzmsg << "[FiveBarClosurePlugin] links of chain '" << _c.motorJointName
          << "' not yet in ECM, retrying.\n";
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// World-frame anchor and axis of a revolute joint. The joint frame is given
// relative to its child link, and gz-sim stores the axis resolved in the
// joint frame.
static bool JointWorldAxis(const gz::sim::Joint& _joint,
                           const gz::sim::Link& _child,
                           const gz::sim::EntityComponentManager& _ecm,
                           gz::math::Vector3d& _origin,
                           gz::math::Vector3d& _axis) {
  auto axes = _joint.Axis(_ecm);
  auto pose = _ecm.Component<gz::sim::components::Pose>(_joint.Entity());
  if (!axes || axes->empty() || !pose) return false;
  const gz::math::Pose3d jointWorld =
      gz::sim::worldPose(_child.Entity(), _ecm) * pose->Data();
  _origin = jointWorld.Pos();
  _axis = jointWorld.Rot().RotateVector((*axes)[0].Xyz()).Normalized();
  return true;
}

// Contribution of one rigid link to a 2-DoF chain mass matrix,
//   M += m Jv^T Jv + Jw^T I Jw,
// where only the first `_nJoints` joints move the link.
static bool AddLinkInertia(const gz::sim::Link& _link,
                           const gz::sim::EntityComponentManager& _ecm,
                           const std::array<gz::math::Vector3d, 2>& _origins,
                           const std::array<gz::math::Vector3d, 2>& _axes,
                           int _nJoints, Mat2& _mass) {
  auto inertial = _ecm.Component<gz::sim::components::Inertial>(_link.Entity());
  if (!inertial) return false;
  const gz::math::Pose3d comWorld =
      gz::sim::worldPose(_link.Entity(), _ecm) * inertial->Data().Pose();
  const gz::math::Matrix3d rot(comWorld.Rot());
  const gz::math::Matrix3d inertiaWorld =
      rot * inertial->Data().MassMatrix().Moi() * rot.Transposed();
  const double m = inertial->Data().MassMatrix().Mass();

  std::array<gz::math::Vector3d, 2> jv, jw;
  for (int j = 0; j < 2; ++j) {
    if (j < _nJoints) {
      jv[j] = _axes[j].Cross(comWorld.Pos() - _origins[j]);
      jw[j] = _axes[j];
    } else {
      jv[j] = jw[j] = gz::math::Vector3d::Zero;
    }
  }
  auto entry = [&](int _i, int _k) {
    return m * jv[_i].Dot(jv[_k]) + jw[_i].Dot(inertiaWorld * jw[_k]);
  };
  _mass = _mass + Mat2{entry(0, 0), entry(0, 1), entry(1, 0), entry(1, 1)};
  return true;
}

bool FiveBarClosurePluginPrivate::UpdateChain(
    FiveBarChain& _c, const gz::sim::EntityComponentManager& _ecm) {
  std::array<gz::math::Vector3d, 2> origins, axes;
  if (!JointWorldAxis(_c.motorJoint, _c.crankLink, _ecm, origins[0], axes[0]) ||
      !JointWorldAxis(_c.elbowJoint, _c.couplerLink, _ecm, origins[1], axes[1]))
    return false;

  _c.p = gz::sim::worldPose(_c.couplerLink.Entity(), _ecm)
             .CoordPositionAdd(_c.tipOffset);
  for (int j = 0; j < 2; ++j) _c.jac[j] = axes[j].Cross(_c.p - origins[j]);

  _c.mass = Mat2{};
  if (!AddLinkInertia(_c.crankLink, _ecm, origins, axes, 1, _c.mass) ||
      !AddLinkInertia(_c.couplerLink, _ecm, origins, axes, 2, _c.mass))
    return false;

  auto v0 = _c.motorJoint.Velocity(_ecm);
  auto v1 = _c.elbowJoint.Velocity(_ecm);
  if (!v0 || v0->empty() || !v1 || v1->empty()) return false;
  _c.qd = {(*v0)[0], (*v1)[0]};
  return true;
}

// ---------------------------------------------------------------------------
// Direct geometric model (five_bar_mgd_spec.md section 2). Only used to put
// the passive joints on the closed-loop manifold once, at start-up.
bool FiveBarClosurePluginPrivate::ComputeElbowAngles(double _theta1,
                                                     double _theta2,
                                                     double& _beta1,
                                                     double& _beta2) {
  const gz::math::Vector2d eL{
      this->a.X() + this->l1 * std::cos(this->phi1 - _theta1),
      this->a.Y() + this->l1 * std::sin(this->phi1 - _theta1)};
  const gz::math::Vector2d eR{
      this->b.X() + this->l1 * std::cos(this->phi2 - _theta2),
      this->b.Y() + this->l1 * std::sin(this->phi2 - _theta2)};
  const gz::math::Vector2d eVec = eR - eL;
  const double dEE = eVec.Length();
  if (dEE < 1e-9 || dEE > 2.0 * this->l2) return false;

  const gz::math::Vector2d u = eVec / dEE;
  const gz::math::Vector2d m = (eL + eR) / 2.0;
  const double h = std::sqrt(this->l2 * this->l2 - (dEE / 2.0) * (dEE / 2.0));
  const gz::math::Vector2d p = m + h * Perp(u);

  const gz::math::Vector2d pMinusEL = p - eL;
  const gz::math::Vector2d pMinusER = p - eR;
  _beta1 = std::atan2(pMinusEL.Y(), pMinusEL.X()) - this->psi1 + _theta1;
  _beta2 = std::atan2(pMinusER.Y(), pMinusER.X()) - this->psi2 + _theta2;
  return true;
}

// ---------------------------------------------------------------------------
void FiveBarClosurePlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                     gz::sim::EntityComponentManager& _ecm) {
  if (_info.paused) return;

  auto& d = *this->dataPtr;

  if (!d.initialized) {
    if (!d.InitChain(d.chain1, _ecm) || !d.InitChain(d.chain2, _ecm)) return;
    d.initialized = true;

    // Assemble the loop once so the constraint starts satisfied; from then
    // on the passive joints are driven only by the constraint force.
    if (d.initializeWithMgd) {
      auto pos1 = d.chain1.motorJoint.Position(_ecm);
      auto pos2 = d.chain2.motorJoint.Position(_ecm);
      const double theta1 = (pos1 && !pos1->empty()) ? (*pos1)[0] : 0.0;
      const double theta2 = (pos2 && !pos2->empty()) ? (*pos2)[0] : 0.0;
      double beta1, beta2;
      if (d.ComputeElbowAngles(theta1, theta2, beta1, beta2)) {
        d.chain1.elbowJoint.ResetPosition(_ecm, {beta1});
        d.chain2.elbowJoint.ResetPosition(_ecm, {beta2});
        gzmsg << "[FiveBarClosurePlugin] Initial assembly: beta1=" << beta1
              << " beta2=" << beta2 << "\n";
      } else {
        gzwarn << "[FiveBarClosurePlugin] Initial motor positions outside "
                  "the workspace; skipping initial assembly.\n";
      }
    }
    gzmsg << "[FiveBarClosurePlugin] Loop-closure constraint force active.\n";
    return;  // Poses are only consistent after the reset has been applied.
  }

  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0) return;
  if (!d.UpdateChain(d.chain1, _ecm) || !d.UpdateChain(d.chain2, _ecm)) return;

  // Constraint c(q) = P1 - P2 = 0, restricted to the mechanism plane (normal
  // to the motor axes): out-of-plane components are carried by the joints.
  gz::math::Vector3d n;
  {
    gz::math::Vector3d origin;
    JointWorldAxis(d.chain1.motorJoint, d.chain1.crankLink, _ecm, origin, n);
  }
  const gz::math::Vector3d e1 = n.Perpendicular().Normalized();
  const gz::math::Vector3d e2 = n.Cross(e1);
  auto toPlane = [&](const gz::math::Vector3d& _v) {
    return gz::math::Vector2d(e1.Dot(_v), e2.Dot(_v));
  };

  std::array<FiveBarChain*, 2> chains{&d.chain1, &d.chain2};
  const std::array<double, 2> sign{1.0, -1.0};

  const gz::math::Vector2d c = toPlane(d.chain1.p - d.chain2.p);

  // Per chain: in-plane Jacobian G_i = sign_i * E^T dP_i/dq_i (2x2), with
  // J = [G_1, G_2]. Build W = J M^-1 J^T, cdot = J qd and J a_free.
  std::array<Mat2, 2> g, massInv;
  Mat2 w;
  gz::math::Vector2d cDot, jAccFree;
  for (int i = 0; i < 2; ++i) {
    const FiveBarChain& ch = *chains[i];
    g[i] = Mat2::FromColumns(sign[i] * toPlane(ch.jac[0]),
                             sign[i] * toPlane(ch.jac[1]));
    if (std::abs(ch.mass.Det()) < 1e-18) return;
    massInv[i] = ch.mass.Inverse();
    w = w + g[i] * massInv[i] * g[i].Transposed();
    cDot += g[i] * ch.qd;

    // Unconstrained joint acceleration, estimated from the last step: the
    // measured acceleration minus the share produced by our own force.
    // This captures motor torques, joint damping, Coriolis terms and
    // contacts without having to model them.
    if (d.havePrev) {
      const gz::math::Vector2d accFree =
          (ch.qd - d.qdPrev[i]) / dt - d.constraintAccPrev[i];
      jAccFree += g[i] * accFree;
    }
  }
  if (std::abs(w.Det()) < 1e-18) return;  // singular (aligned) configuration

  // Lagrange multiplier = force transmitted at P, chosen so that after this
  // step J qd+ = -baumgarte * c / dt (velocity-level Baumgarte).
  gz::math::Vector2d lambda =
      w.Inverse() * (-d.baumgarte / (dt * dt) * c - cDot / dt - jAccFree);

  const double lambdaNorm = lambda.Length();
  if (lambdaNorm > d.maxForce) {
    lambda *= d.maxForce / lambdaNorm;
    if (!d.clamping) {
      gzwarn << "[FiveBarClosurePlugin] Constraint force " << lambdaNorm
             << " N clamped to max_force=" << d.maxForce
             << " N; |c|=" << c.Length() << " m.\n";
      d.clamping = true;
    }
  } else if (d.clamping) {
    gzmsg << "[FiveBarClosurePlugin] Constraint force back below max_force.\n";
    d.clamping = false;
  }

  // Apply +F at P1 on coupler 1 and -F at P2 on coupler 2. AddWorldForce
  // takes the application point relative to the link center of mass.
  const gz::math::Vector3d force = lambda.X() * e1 + lambda.Y() * e2;
  for (int i = 0; i < 2; ++i) {
    FiveBarChain& ch = *chains[i];
    auto inertial =
        _ecm.Component<gz::sim::components::Inertial>(ch.couplerLink.Entity());
    ch.couplerLink.AddWorldForce(_ecm, sign[i] * force,
                                 ch.tipOffset - inertial->Data().Pose().Pos());

    d.qdPrev[i] = ch.qd;
    d.constraintAccPrev[i] = massInv[i] * (g[i].Transposed() * lambda);
  }
  d.havePrev = true;

  if (_info.simTime - d.lastLog >= std::chrono::seconds(1)) {
    d.lastLog = _info.simTime;
    gzdbg << "[FiveBarClosurePlugin] |c|=" << c.Length()
          << " m  |F|=" << lambda.Length() << " N  P1=" << d.chain1.p << "\n";
  }
}

}  // namespace motkin_gz

// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(motkin_gz::FiveBarClosurePlugin, gz::sim::System,
              motkin_gz::FiveBarClosurePlugin::ISystemConfigure,
              motkin_gz::FiveBarClosurePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(motkin_gz::FiveBarClosurePlugin,
                    "motkin_gz::FiveBarClosurePlugin")
