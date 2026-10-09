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

#include "motkin_forward_command_controller/motkin_forward_command_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace motkin_forward_command_controller {

static constexpr const char* HW_IF_GAIN_KP = "gain_kp";
static constexpr const char* HW_IF_GAIN_KD = "gain_kd";
static constexpr const char* HW_IF_CLOCK = "clock";
static constexpr const char* HW_IF_INDEX = "latest_command_index";
static constexpr const char* HW_IF_FLAGS = "flags";

// Joint state interfaces published on ~/board_state, in the order of
// JointStateRefs.
static const std::array<std::string, kNumJointStateFields> JOINT_STATE_FIELDS{
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT, HW_IF_GAIN_KP, HW_IF_GAIN_KD};

MotkinForwardCommandController::MotkinForwardCommandController()
    : controller_interface::ControllerInterface(),
      joints_command_subscriber_(nullptr) {}

controller_interface::CallbackReturn MotkinForwardCommandController::on_init() {
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
MotkinForwardCommandController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();
  joint_names_ = params_.joints;

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  pos_interface_names_.clear();
  vel_interface_names_.clear();
  eff_interface_names_.clear();
  kp_interface_names_.clear();
  kd_interface_names_.clear();

  for (const auto& joint : joint_names_) {
    pos_interface_names_.push_back(joint + "/" +
                                   hardware_interface::HW_IF_POSITION);
    vel_interface_names_.push_back(joint + "/" +
                                   hardware_interface::HW_IF_VELOCITY);
    eff_interface_names_.push_back(joint + "/" +
                                   hardware_interface::HW_IF_EFFORT);
    kp_interface_names_.push_back(joint + "/" + HW_IF_GAIN_KP);
    kd_interface_names_.push_back(joint + "/" + HW_IF_GAIN_KD);
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { rt_command_.set(*msg); });

  board_state_publisher_ = get_node()->create_publisher<BoardStateMsg>(
      "~/board_state", rclcpp::SystemDefaultsQoS());
  rt_board_state_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<BoardStateMsg>>(
          board_state_publisher_);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotkinForwardCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.insert(config.names.end(), pos_interface_names_.begin(),
                      pos_interface_names_.end());
  config.names.insert(config.names.end(), vel_interface_names_.begin(),
                      vel_interface_names_.end());
  config.names.insert(config.names.end(), eff_interface_names_.begin(),
                      eff_interface_names_.end());
  config.names.insert(config.names.end(), kp_interface_names_.begin(),
                      kp_interface_names_.end());
  config.names.insert(config.names.end(), kd_interface_names_.begin(),
                      kd_interface_names_.end());
  return config;
}

controller_interface::InterfaceConfiguration
MotkinForwardCommandController::state_interface_configuration() const {
  // Read all the state interfaces of the robot to publish them on
  // ~/board_state.
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn
MotkinForwardCommandController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const std::size_t n = joint_names_.size();

  RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu interfaces", n);

  auto assign_interfaces = [&](const std::vector<std::string>& names,
                               std::vector<LoanedRef>& out) -> bool {
    std::vector<LoanedRef> tmp;
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, names, std::string(""), tmp) ||
        tmp.size() != n) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected %zu interfaces for '%s', got %zu", n,
                   names.empty()
                       ? "?"
                       : names[0].substr(names[0].rfind('/') + 1).c_str(),
                   tmp.size());
      return false;
    }
    out = std::move(tmp);
    return true;
  };

  if (!assign_interfaces(pos_interface_names_, pos_interfaces_) ||
      !assign_interfaces(vel_interface_names_, vel_interfaces_) ||
      !assign_interfaces(eff_interface_names_, eff_interfaces_) ||
      !assign_interfaces(kp_interface_names_, kp_interfaces_) ||
      !assign_interfaces(kd_interface_names_, kd_interfaces_)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Seed the command buffer from the per-joint YAML initial_command
  // parameters, using the same [pos×n | vel×n | eff×n | gain_kp×n |
  // gain_kd×n] layout as ~/commands, so joints hold a known state before
  // the first message arrives instead of being left uncommanded.
  joint_commands_ = CmdType{};
  joint_commands_.data.resize(5 * n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& initial =
        params_.initial_command.joints_map.at(joint_names_[i]);
    joint_commands_.data[0 * n + i] = initial.position;
    joint_commands_.data[1 * n + i] = initial.velocity;
    joint_commands_.data[2 * n + i] = initial.effort;
    joint_commands_.data[3 * n + i] = initial.gain_kp;
    joint_commands_.data[4 * n + i] = initial.gain_kd;
  }
  rt_command_.set(joint_commands_);

  // Sort the state interfaces into the BoardState fields. Controlled joints
  // come first, in the order of the joints parameter, then the other joints in
  // the order the hardware exports them.
  std::vector<std::string> state_joint_names = joint_names_;
  joint_state_interfaces_.assign(n, JointStateRefs{});
  gpio_clock_ = gpio_index_ = gpio_flags_ = nullptr;

  auto expect_type = [&](const hardware_interface::LoanedStateInterface& iface,
                         hardware_interface::HandleDataType type) {
    if (iface.get_data_type() == type) {
      return &iface;
    }
    RCLCPP_WARN(get_node()->get_logger(),
                "State interface '%s' has data type '%s', expected '%s': it "
                "will not be published",
                iface.get_name().c_str(),
                iface.get_data_type().to_string().c_str(),
                type.to_string().c_str());
    return static_cast<const hardware_interface::LoanedStateInterface*>(
        nullptr);
  };

  for (const auto& iface : state_interfaces_) {
    const auto& prefix = iface.get_prefix_name();
    const auto& name = iface.get_interface_name();

    if (prefix == params_.gpio_name) {
      if (name == HW_IF_CLOCK) {
        gpio_clock_ =
            expect_type(iface, hardware_interface::HandleDataType::UINT32);
      } else if (name == HW_IF_INDEX) {
        gpio_index_ =
            expect_type(iface, hardware_interface::HandleDataType::UINT32);
      } else if (name == HW_IF_FLAGS) {
        gpio_flags_ =
            expect_type(iface, hardware_interface::HandleDataType::UINT8);
      }
      continue;
    }

    const auto field =
        std::find(JOINT_STATE_FIELDS.begin(), JOINT_STATE_FIELDS.end(), name);
    if (field == JOINT_STATE_FIELDS.end()) {
      continue;
    }
    auto joint =
        std::find(state_joint_names.begin(), state_joint_names.end(), prefix);
    if (joint == state_joint_names.end()) {
      state_joint_names.push_back(prefix);
      joint_state_interfaces_.emplace_back();
      joint = std::prev(state_joint_names.end());
    }
    joint_state_interfaces_[joint - state_joint_names.begin()]
                           [field - JOINT_STATE_FIELDS.begin()] = expect_type(
                               iface,
                               hardware_interface::HandleDataType::DOUBLE);
  }

  // Preallocate the message so that update() does not allocate.
  const std::size_t n_state = state_joint_names.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  board_state_msg_ = BoardStateMsg{};
  board_state_msg_.name = state_joint_names;
  board_state_msg_.position.assign(n_state, nan);
  board_state_msg_.velocity.assign(n_state, nan);
  board_state_msg_.effort.assign(n_state, nan);
  board_state_msg_.gain_kp.assign(n_state, nan);
  board_state_msg_.gain_kd.assign(n_state, nan);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MotkinForwardCommandController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  pos_interfaces_.clear();
  vel_interfaces_.clear();
  eff_interfaces_.clear();
  kp_interfaces_.clear();
  kd_interfaces_.clear();
  joint_state_interfaces_.clear();
  gpio_clock_ = gpio_index_ = gpio_flags_ = nullptr;
  return controller_interface::CallbackReturn::SUCCESS;
}

void MotkinForwardCommandController::publish_board_state(
    const rclcpp::Time& time) {
  // A value that cannot be read this cycle keeps its previous value.
  auto read = [](const hardware_interface::LoanedStateInterface* iface,
                 auto& out) {
    using T = std::decay_t<decltype(out)>;
    if (iface != nullptr) {
      if (const auto value = iface->get_optional<T>(); value.has_value()) {
        out = *value;
      }
    }
  };

  board_state_msg_.header.stamp = time;
  std::array<std::vector<double>*, kNumJointStateFields> fields{
      &board_state_msg_.position, &board_state_msg_.velocity,
      &board_state_msg_.effort, &board_state_msg_.gain_kp,
      &board_state_msg_.gain_kd};
  for (std::size_t i = 0; i < joint_state_interfaces_.size(); ++i) {
    for (std::size_t f = 0; f < kNumJointStateFields; ++f) {
      read(joint_state_interfaces_[i][f], (*fields[f])[i]);
    }
  }
  read(gpio_clock_, board_state_msg_.clock);
  read(gpio_index_, board_state_msg_.latest_command_index);
  read(gpio_flags_, board_state_msg_.flags);

  rt_board_state_publisher_->try_publish(board_state_msg_);
}

controller_interface::return_type MotkinForwardCommandController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  publish_board_state(time);

  auto cmd_op = rt_command_.try_get();
  if (cmd_op.has_value()) {
    joint_commands_ = cmd_op.value();
  }

  const std::size_t n = joint_names_.size();
  const auto& data = joint_commands_.data;

  RCLCPP_DEBUG(get_node()->get_logger(), " data.size()= %ld n= %ld",
               data.size(), n);
  // Require exactly 5*n values: [pos×n | vel×n | eff×n | gain_kp×n | gain_kd×n]
  if (data.size() != 5 * n) {
    return controller_interface::return_type::OK;
  }

  auto apply = [&](std::size_t offset, std::vector<LoanedRef>& ifaces) {
    std::size_t local_type = offset % n;
    for (std::size_t i = 0; i < n; ++i) {
      if (std::isfinite(data[offset + i]) &&
          !ifaces[i].get().set_value(data[offset + i])) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                             *(get_node()->get_clock()), 1000,
                             "Failed to set command interface '%s'",
                             ifaces[i].get().get_name().c_str());
      }
      RCLCPP_DEBUG(get_node()->get_logger(), "%ld %f", local_type,
                   data[offset + i]);
    }
  };

  apply(0 * n, pos_interfaces_);
  apply(1 * n, vel_interfaces_);
  apply(2 * n, eff_interfaces_);
  apply(3 * n, kp_interfaces_);
  apply(4 * n, kd_interfaces_);

  return controller_interface::return_type::OK;
}

}  // namespace motkin_forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    motkin_forward_command_controller::MotkinForwardCommandController,
    controller_interface::ControllerInterface)
