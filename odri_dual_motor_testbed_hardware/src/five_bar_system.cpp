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

#include "odri_dual_motor_testbed_hardware/five_bar_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// The wrapped plugin may still use the legacy export_*_interfaces() API (the
// pico_dual_drv8316c one does), so this wrapper forwards through it.
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace odri_dual_motor_testbed_hardware {

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

constexpr double kTwoPi = 2.0 * M_PI;

std::string GetParam(const hardware_interface::HardwareInfo& info,
                     const std::string& key, const std::string& default_value) {
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? default_value : it->second;
}

bool GetDouble(const hardware_interface::HardwareInfo& info,
               const std::string& key, double& value,
               const rclcpp::Logger& logger) {
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return true;
  try {
    value = std::stod(it->second);
    return true;
  } catch (const std::exception&) {
    RCLCPP_FATAL(logger, "Hardware parameter '%s'='%s' is not a number.",
                 key.c_str(), it->second.c_str());
    return false;
  }
}

double Unwrap(double value, double previous) {
  return value + kTwoPi * std::round((previous - value) / kTwoPi);
}

}  // namespace

FiveBarSystem::FiveBarSystem() = default;

FiveBarSystem::~FiveBarSystem() {
  inner_.reset();
  loader_.reset();
}

CallbackReturn FiveBarSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto logger = get_logger();

  motor_names_[0] = GetParam(info_, "motor_joint1", "motor_1");
  motor_names_[1] = GetParam(info_, "motor_joint2", "motor_2");
  passive_[0].name = GetParam(info_, "passive_joint1", "passive_1");
  passive_[1].name = GetParam(info_, "passive_joint2", "passive_2");

  auto& g = geometry_;
  if (!GetDouble(info_, "a_x", g.a_x, logger) ||
      !GetDouble(info_, "a_y", g.a_y, logger) ||
      !GetDouble(info_, "b_x", g.b_x, logger) ||
      !GetDouble(info_, "b_y", g.b_y, logger) ||
      !GetDouble(info_, "l1", g.l1, logger) ||
      !GetDouble(info_, "l2", g.l2, logger) ||
      !GetDouble(info_, "phi1", g.phi1, logger) ||
      !GetDouble(info_, "phi2", g.phi2, logger) ||
      !GetDouble(info_, "psi1", g.psi1, logger) ||
      !GetDouble(info_, "psi2", g.psi2, logger)) {
    return CallbackReturn::ERROR;
  }

  // Split the joints: passive ones stay here, all others go to the inner
  // plugin.
  inner_info_ = info_;
  inner_info_.joints.clear();
  bool found[2] = {false, false};
  for (const auto& joint : info_.joints) {
    int idx = -1;
    for (int i = 0; i < 2; ++i) {
      if (joint.name == passive_[i].name) idx = i;
    }
    if (idx < 0) {
      inner_info_.joints.push_back(joint);
      continue;
    }
    if (!joint.command_interfaces.empty()) {
      RCLCPP_FATAL(logger,
                   "Passive joint '%s' must not have command "
                   "interfaces.",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto& state_if : joint.state_interfaces) {
      if (state_if.name == hardware_interface::HW_IF_POSITION) {
        passive_[idx].has_position = true;
      } else if (state_if.name == hardware_interface::HW_IF_VELOCITY) {
        passive_[idx].has_velocity = true;
      } else if (state_if.name == hardware_interface::HW_IF_EFFORT) {
        passive_[idx].has_effort = true;
      } else {
        RCLCPP_FATAL(logger,
                     "Passive joint '%s': unsupported state "
                     "interface '%s' (position, velocity, effort only).",
                     joint.name.c_str(), state_if.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    found[idx] = true;
  }
  for (int i = 0; i < 2; ++i) {
    if (!found[i]) {
      RCLCPP_FATAL(logger,
                   "Passive joint '%s' is not declared under "
                   "<ros2_control>.",
                   passive_[i].name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  const std::string inner_plugin = GetParam(
      info_, "inner_plugin",
      "pico_dual_drv8316c_hardware_interface/SystemPicoDualDrv8316CHardware");
  inner_info_.hardware_plugin_name = inner_plugin;
  inner_info_.name = info_.name + "_inner";
  // The wrapper's own read()/write() drive the inner plugin synchronously.
  inner_info_.is_async = false;

  try {
    loader_ = std::make_unique<
        pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
        "hardware_interface", "hardware_interface::SystemInterface");
    inner_ = loader_->createUniqueInstance(inner_plugin);
  } catch (const pluginlib::PluginlibException& ex) {
    RCLCPP_FATAL(logger, "Cannot load inner plugin '%s': %s",
                 inner_plugin.c_str(), ex.what());
    return CallbackReturn::ERROR;
  }

  hardware_interface::HardwareComponentParams inner_params;
  inner_params.hardware_info = inner_info_;
  inner_params.logger = logger;
  inner_params.clock = get_clock();
  // No executor: the inner plugin must not create a second node with our
  // name.
  if (inner_->init(inner_params) != CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(logger, "Inner plugin '%s' failed to initialize.",
                 inner_plugin.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger,
              "Five-bar wrapper around '%s': motors (%s, %s) -> passive "
              "(%s, %s); A=(%g, %g) B=(%g, %g) L1=%g L2=%g phi=(%g, %g) "
              "psi=(%g, %g)",
              inner_plugin.c_str(), motor_names_[0].c_str(),
              motor_names_[1].c_str(), passive_[0].name.c_str(),
              passive_[1].name.c_str(), g.a_x, g.a_y, g.b_x, g.b_y, g.l1, g.l2,
              g.phi1, g.phi2, g.psi1, g.psi2);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FiveBarSystem::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  have_passive_positions_ = false;
  out_of_workspace_ = false;
  return inner_->on_configure(previous_state);
}

CallbackReturn FiveBarSystem::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  return inner_->on_cleanup(previous_state);
}

CallbackReturn FiveBarSystem::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  return inner_->on_shutdown(previous_state);
}

CallbackReturn FiveBarSystem::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  return inner_->on_activate(previous_state);
}

CallbackReturn FiveBarSystem::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return inner_->on_deactivate(previous_state);
}

CallbackReturn FiveBarSystem::on_error(
    const rclcpp_lifecycle::State& previous_state) {
  return inner_->on_error(previous_state);
}

void FiveBarSystem::FindMotorHandles(
    const std::vector<hardware_interface::StateInterface::ConstSharedPtr>&
        state_interfaces) {
  for (const auto& si : state_interfaces) {
    for (int i = 0; i < 2; ++i) {
      if (si->get_prefix_name() != motor_names_[i]) continue;
      if (si->get_interface_name() == hardware_interface::HW_IF_POSITION) {
        motor_position_[i] = si;
      } else if (si->get_interface_name() ==
                 hardware_interface::HW_IF_VELOCITY) {
        motor_velocity_[i] = si;
      }
    }
  }
  for (int i = 0; i < 2; ++i) {
    // Returning without this handle would leave the passive joints frozen,
    // so fail loudly instead.
    if (!motor_position_[i]) {
      throw std::runtime_error("FiveBarSystem: inner plugin does not export '" +
                               motor_names_[i] +
                               "/position'; cannot compute the passive "
                               "joints.");
    }
  }
}

std::vector<hardware_interface::StateInterface>
FiveBarSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces =
      inner_->export_state_interfaces();
  // Empty: the inner plugin uses the framework-managed API, and the framework
  // will call on_export_state_interfaces() next.
  if (state_interfaces.empty()) return {};

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> copies;
  for (const auto& si : state_interfaces) {
    // Legacy handles only hold a pointer into the inner plugin's storage, so
    // a copy reads the same value.
    copies.push_back(
        std::make_shared<const hardware_interface::StateInterface>(si));
  }
  FindMotorHandles(copies);

  for (auto& p : passive_) {
    if (p.has_position) {
      state_interfaces.emplace_back(p.name, hardware_interface::HW_IF_POSITION,
                                    &p.position);
    }
    if (p.has_velocity) {
      state_interfaces.emplace_back(p.name, hardware_interface::HW_IF_VELOCITY,
                                    &p.velocity);
    }
    if (p.has_effort) {
      state_interfaces.emplace_back(p.name, hardware_interface::HW_IF_EFFORT,
                                    &p.effort);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
FiveBarSystem::on_export_state_interfaces() {
  auto state_interfaces = inner_->on_export_state_interfaces();
  FindMotorHandles(state_interfaces);

  auto add = [&](const std::string& joint, const char* interface,
                 const double* value) {
    hardware_interface::InterfaceInfo info{};
    info.name = interface;
    info.data_type = "double";
    auto handle = std::make_shared<hardware_interface::StateInterface>(
        hardware_interface::InterfaceDescription(joint, info));
    passive_handles_.emplace_back(handle, value);
    state_interfaces.push_back(handle);
  };
  for (const auto& p : passive_) {
    if (p.has_position)
      add(p.name, hardware_interface::HW_IF_POSITION, &p.position);
    if (p.has_velocity)
      add(p.name, hardware_interface::HW_IF_VELOCITY, &p.velocity);
    if (p.has_effort) add(p.name, hardware_interface::HW_IF_EFFORT, &p.effort);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FiveBarSystem::export_command_interfaces() {
  return inner_->export_command_interfaces();
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
FiveBarSystem::on_export_command_interfaces() {
  return inner_->on_export_command_interfaces();
}

return_type FiveBarSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  return inner_->prepare_command_mode_switch(start_interfaces, stop_interfaces);
}

return_type FiveBarSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  return inner_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

void FiveBarSystem::PushPassiveHandles() {
  for (auto& [handle, value] : passive_handles_) {
    (void)handle->set_value(*value);
  }
}

return_type FiveBarSystem::read(const rclcpp::Time& time,
                                const rclcpp::Duration& period) {
  const return_type ret = inner_->read(time, period);
  if (ret != return_type::OK) return ret;

  double theta[2];
  double theta_dot[2] = {0.0, 0.0};
  for (int i = 0; i < 2; ++i) {
    const auto pos = motor_position_[i]->get_optional();
    if (!pos) return return_type::OK;  // keep the previous passive values
    theta[i] = *pos;
    if (motor_velocity_[i]) {
      theta_dot[i] = motor_velocity_[i]->get_optional().value_or(0.0);
    }
  }

  PassiveJointState s;
  const FiveBarStatus status = ComputePassiveJoints(
      theta[0], theta[1], theta_dot[0], theta_dot[1], geometry_, s);
  if (status == FiveBarStatus::kOutOfWorkspace) {
    if (!out_of_workspace_) {
      RCLCPP_ERROR(get_logger(),
                   "(%s, %s) = (%.4f, %.4f) is outside the five-bar "
                   "workspace; passive joints hold their last value.",
                   motor_names_[0].c_str(), motor_names_[1].c_str(), theta[0],
                   theta[1]);
      out_of_workspace_ = true;
    }
    passive_[0].velocity = 0.0;
    passive_[1].velocity = 0.0;
    PushPassiveHandles();
    return return_type::OK;
  }
  if (out_of_workspace_) {
    RCLCPP_INFO(get_logger(), "Back inside the five-bar workspace.");
    out_of_workspace_ = false;
  }
  if (status == FiveBarStatus::kSingularVelocity) {
    RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 1000,
                         "Five-bar at a singular configuration; passive "
                         "joint velocities set to 0.");
  }

  if (have_passive_positions_) {
    s.beta1 = Unwrap(s.beta1, passive_[0].position);
    s.beta2 = Unwrap(s.beta2, passive_[1].position);
  }
  passive_[0].position = s.beta1;
  passive_[1].position = s.beta2;
  passive_[0].velocity = s.beta1_dot;
  passive_[1].velocity = s.beta2_dot;
  have_passive_positions_ = true;
  PushPassiveHandles();
  return return_type::OK;
}

return_type FiveBarSystem::write(const rclcpp::Time& time,
                                 const rclcpp::Duration& period) {
  return inner_->write(time, period);
}

}  // namespace odri_dual_motor_testbed_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odri_dual_motor_testbed_hardware::FiveBarSystem,
                       hardware_interface::SystemInterface)
