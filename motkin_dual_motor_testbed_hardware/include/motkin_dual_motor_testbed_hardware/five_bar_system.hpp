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

#ifndef MOTKIN_DUAL_MOTOR_TESTBED_HARDWARE__FIVE_BAR_SYSTEM_HPP_
#define MOTKIN_DUAL_MOTOR_TESTBED_HARDWARE__FIVE_BAR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motkin_dual_motor_testbed_hardware/five_bar_passive_joints.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motkin_dual_motor_testbed_hardware {

/// \brief SystemInterface exposing the passive joints of the five-bar.
///
/// Only motor_1 / motor_2 have encoders. This plugin loads the hardware
/// plugin that drives them (hardware parameter `inner_plugin`, by default
/// the motkin board), forwards every lifecycle transition,
/// read(), write() and command mode switch to it, and additionally exports
/// the passive joints as state-only interfaces (position, velocity, effort;
/// effort is always 0). Their values are recomputed after each inner read()
/// with the exact direct geometric model of five_bar_passive_joints.hpp, so
/// joint_state_broadcaster publishes the full five-bar.
///
/// The inner plugin receives the same HardwareInfo minus the passive joints.
///
/// Hardware parameters (all optional, shown with defaults):
///   inner_plugin    motkin_ros2_hardware_interface/
///                   SystemPicoDualDrv8316CHardware
///   motor_joint1    motor_1      passive_joint1  passive_1
///   motor_joint2    motor_2      passive_joint2  passive_2
///   a_x a_y b_x b_y l1 l2 phi1 phi2 psi1 psi2   see FiveBarGeometry
class FiveBarSystem : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FiveBarSystem)

  FiveBarSystem();
  ~FiveBarSystem() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params)
      override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  // Both export APIs are forwarded, so the inner plugin may use either: the
  // legacy one (raw pointers, e.g. motkin) or the
  // framework-managed one (e.g. mock_components/GenericSystem).
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct PassiveJoint {
    std::string name;
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    bool has_position{false};
    bool has_velocity{false};
    bool has_effort{false};
  };

  // Declared before inner_ so that the instance is destroyed first.
  std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>>
      loader_;
  pluginlib::UniquePtr<hardware_interface::SystemInterface> inner_;
  hardware_interface::HardwareInfo inner_info_;

  FiveBarGeometry geometry_;
  std::string motor_names_[2];
  PassiveJoint passive_[2];

  // The inner plugin's motor state handles, read after each read().
  hardware_interface::StateInterface::ConstSharedPtr motor_position_[2];
  hardware_interface::StateInterface::ConstSharedPtr motor_velocity_[2];

  // Framework-managed export only: passive handles and the value each mirrors.
  std::vector<
      std::pair<hardware_interface::StateInterface::SharedPtr, const double*>>
      passive_handles_;

  void FindMotorHandles(
      const std::vector<hardware_interface::StateInterface::ConstSharedPtr>&
          state_interfaces);
  void PushPassiveHandles();

  // Once set, passive positions are unwrapped against the previous value so
  // they stay continuous across the atan2 branch cut.
  bool have_passive_positions_{false};
  bool out_of_workspace_{false};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
};

}  // namespace motkin_dual_motor_testbed_hardware

#endif  // MOTKIN_DUAL_MOTOR_TESTBED_HARDWARE__FIVE_BAR_SYSTEM_HPP_
