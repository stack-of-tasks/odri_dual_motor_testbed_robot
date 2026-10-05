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

#include "motkin_five_bar_force_velocity_controller/motkin_five_bar_force_velocity_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace motkin_five_bar_force_velocity_controller {

static constexpr const char* HW_IF_GAIN_KP = "gain_kp";
static constexpr const char* HW_IF_GAIN_KD = "gain_kd";

MotkinFiveBarForceVelocityController::MotkinFiveBarForceVelocityController()
    : controller_interface::ControllerInterface(),
      contact_force_subscriber_(nullptr) {}

controller_interface::CallbackReturn
MotkinFiveBarForceVelocityController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MotkinFiveBarForceVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();

  left_joint_name_ = params_.motor_left_joint;
  right_joint_name_ = params_.motor_right_joint;

  geometry_.a_x = params_.geometry.a_x;
  geometry_.a_z = params_.geometry.a_z;
  geometry_.b_x = params_.geometry.b_x;
  geometry_.b_z = params_.geometry.b_z;
  geometry_.l1 = params_.geometry.l1;
  geometry_.l2 = params_.geometry.l2;
  geometry_.phi1 = params_.geometry.phi1;
  geometry_.phi2 = params_.geometry.phi2;

  contact_force_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          params_.contact_force_topic, rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            rt_contact_force_.set(*msg);
          });

  RCLCPP_INFO(get_node()->get_logger(),
              "configure successful (left=%s, right=%s, contact_force=%s)",
              left_joint_name_.c_str(), right_joint_name_.c_str(),
              params_.contact_force_topic.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotkinFiveBarForceVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : {left_joint_name_, right_joint_name_}) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto& joint : {left_joint_name_, right_joint_name_}) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto& joint : {left_joint_name_, right_joint_name_}) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  for (const auto& joint : {left_joint_name_, right_joint_name_}) {
    config.names.push_back(joint + "/" + HW_IF_GAIN_KP);
  }
  for (const auto& joint : {left_joint_name_, right_joint_name_}) {
    config.names.push_back(joint + "/" + HW_IF_GAIN_KD);
  }
  return config;
}

controller_interface::InterfaceConfiguration
MotkinFiveBarForceVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : {left_joint_name_, right_joint_name_}) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::CallbackReturn
MotkinFiveBarForceVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const std::vector<std::string> joint_names{left_joint_name_,
                                             right_joint_name_};

  auto assign_cmd = [&](const std::string& interface_name,
                        std::vector<LoanedCmdRef>& out) -> bool {
    std::vector<std::string> names;
    for (const auto& joint : joint_names) {
      names.push_back(joint + "/" + interface_name);
    }
    std::vector<LoanedCmdRef> tmp;
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, names, std::string(""), tmp) ||
        tmp.size() != joint_names.size()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected %zu command interfaces for '%s', got %zu",
                   joint_names.size(), interface_name.c_str(), tmp.size());
      return false;
    }
    out = std::move(tmp);
    return true;
  };

  auto assign_state = [&](const std::string& interface_name,
                          std::vector<LoanedStateRef>& out) -> bool {
    std::vector<std::string> names;
    for (const auto& joint : joint_names) {
      names.push_back(joint + "/" + interface_name);
    }
    std::vector<LoanedStateRef> tmp;
    if (!controller_interface::get_ordered_interfaces(state_interfaces_, names,
                                                      std::string(""), tmp) ||
        tmp.size() != joint_names.size()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected %zu state interfaces for '%s', got %zu",
                   joint_names.size(), interface_name.c_str(), tmp.size());
      return false;
    }
    out = std::move(tmp);
    return true;
  };

  if (!assign_cmd(hardware_interface::HW_IF_POSITION, pos_cmd_) ||
      !assign_cmd(hardware_interface::HW_IF_VELOCITY, vel_cmd_) ||
      !assign_cmd(hardware_interface::HW_IF_EFFORT, eff_cmd_) ||
      !assign_cmd(HW_IF_GAIN_KP, kp_cmd_) ||
      !assign_cmd(HW_IF_GAIN_KD, kd_cmd_) ||
      !assign_state(hardware_interface::HW_IF_POSITION, pos_state_)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MotkinFiveBarForceVelocityController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  pos_cmd_.clear();
  vel_cmd_.clear();
  eff_cmd_.clear();
  kp_cmd_.clear();
  kd_cmd_.clear();
  pos_state_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotkinFiveBarForceVelocityController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  const double theta1 = pos_state_[0].get().get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  const double theta2 = pos_state_[1].get().get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());

  double fx = 0.0;
  double fy = 0.0;
  auto wrench_op = rt_contact_force_.try_get();
  if (wrench_op.has_value()) {
    fx = wrench_op->wrench.force.x;
    fy = wrench_op->wrench.force.y;
  }

  double qdot1 = 0.0;
  double qdot2 = 0.0;
  if (std::isfinite(theta1) && std::isfinite(theta2)) {
    try {
      const Vec2 qdot =
          JointVelocityFromContactForce(theta1, theta2, fx, fy, geometry_);
      qdot1 = qdot[0];
      qdot2 = qdot[1];
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "qdot = J^T f_c unavailable, holding still: %s", e.what());
      qdot1 = 0.0;
      qdot2 = 0.0;
    }
  }

  const double v_max = params_.max_joint_velocity;
  qdot1 = std::clamp(qdot1, -v_max, v_max);
  qdot2 = std::clamp(qdot2, -v_max, v_max);

  double tau_g1 = 0.0;
  double tau_g2 = 0.0;
  if (params_.gravity_compensation_enabled && std::isfinite(theta1) &&
      std::isfinite(theta2)) {
    const Vec2 tau_g =
        GravityTorque(theta1, theta2, geometry_, params_.coupler_mass_left,
                      params_.coupler_mass_right, params_.gravity);
    tau_g1 = tau_g[0];
    tau_g2 = tau_g[1];
  }

  const std::array<double, 2> theta{theta1, theta2};
  const std::array<double, 2> qdot{qdot1, qdot2};
  const std::array<double, 2> tau_g{tau_g1, tau_g2};

  auto warn_if_failed = [this](bool ok, const char* interface_name) {
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()),
                           1000, "Failed to set command interface '%s'",
                           interface_name);
    }
  };

  for (std::size_t i = 0; i < 2; ++i) {
    if (std::isfinite(theta[i])) {
      warn_if_failed(pos_cmd_[i].get().set_value(theta[i]), "position");
    }
    warn_if_failed(vel_cmd_[i].get().set_value(qdot[i]), "velocity");
    warn_if_failed(eff_cmd_[i].get().set_value(tau_g[i]), "effort");
    warn_if_failed(kp_cmd_[i].get().set_value(params_.gain_kp), "gain_kp");
    warn_if_failed(kd_cmd_[i].get().set_value(params_.gain_kd), "gain_kd");
  }

  return controller_interface::return_type::OK;
}

}  // namespace motkin_five_bar_force_velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(motkin_five_bar_force_velocity_controller::
                           MotkinFiveBarForceVelocityController,
                       controller_interface::ControllerInterface)
