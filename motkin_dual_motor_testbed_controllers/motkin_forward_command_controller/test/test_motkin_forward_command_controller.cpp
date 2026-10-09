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

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/test_utils.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "motkin_forward_command_controller/motkin_forward_command_controller.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using controller_interface::activate_succeeds;
using controller_interface::configure_succeeds;
using controller_interface::deactivate_succeeds;
using hardware_interface::CommandInterface;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using hardware_interface::StateInterface;

static constexpr const char* HW_IF_GAIN_KP = "gain_kp";
static constexpr const char* HW_IF_GAIN_KD = "gain_kd";

// Re-exports protected members as public so tests can inject commands and
// inspect state.
class FriendController
    : public motkin_forward_command_controller::MotkinForwardCommandController {
 public:
  using MotkinForwardCommandController::board_state_msg_;
  using MotkinForwardCommandController::board_state_publisher_;
  using MotkinForwardCommandController::eff_interfaces_;
  using MotkinForwardCommandController::kd_interfaces_;
  using MotkinForwardCommandController::kp_interfaces_;
  using MotkinForwardCommandController::pos_interfaces_;
  using MotkinForwardCommandController::rt_command_;
  using MotkinForwardCommandController::vel_interfaces_;
};

class MotkinForwardCommandControllerTest : public ::testing::Test {
 public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() { controller_ = std::make_unique<FriendController>(); }
  void TearDown() { controller_.reset(); }

  // Initialise controller and wire up all 5 command interfaces per joint.
  // joints is passed via node_options so generate_parameter_library sees it at
  // init time.
  void SetUpController(
      const std::vector<std::string>& joints = {"j1", "j2"},
      std::vector<StateInterface::ConstSharedPtr> state_ifs = {}) {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "motkin_forward_command_controller";
    params.robot_description = "";
    params.controller_manager_update_rate = 100;
    params.node_namespace = "";
    auto node_opts = controller_->define_custom_node_options();
    node_opts.append_parameter_override("joints", joints);
    params.node_options = node_opts;
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);

    std::vector<LoanedCommandInterface> cmd_ifs;
    cmd_ifs.emplace_back(j1_pos_);
    cmd_ifs.emplace_back(j1_vel_);
    cmd_ifs.emplace_back(j1_eff_);
    cmd_ifs.emplace_back(j1_kp_);
    cmd_ifs.emplace_back(j1_kd_);
    cmd_ifs.emplace_back(j2_pos_);
    cmd_ifs.emplace_back(j2_vel_);
    cmd_ifs.emplace_back(j2_eff_);
    cmd_ifs.emplace_back(j2_kp_);
    cmd_ifs.emplace_back(j2_kd_);
    std::vector<LoanedStateInterface> loaned_state_ifs;
    for (const auto& state_if : state_ifs) {
      loaned_state_ifs.emplace_back(state_if);
    }
    controller_->assign_interfaces(std::move(cmd_ifs),
                                   std::move(loaned_state_ifs));
    executor_.add_node(controller_->get_node()->get_node_base_interface());
  }

 protected:
  std::unique_ptr<FriendController> controller_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // backing storage — two joints
  double j1_pos_val_{0.0}, j1_vel_val_{0.0}, j1_eff_val_{0.0}, j1_kp_val_{0.0},
      j1_kd_val_{0.0};
  double j2_pos_val_{0.0}, j2_vel_val_{0.0}, j2_eff_val_{0.0}, j2_kp_val_{0.0},
      j2_kd_val_{0.0};

  // Named CommandInterface objects (LoanedCommandInterface requires non-const
  // lvalue ref)
  CommandInterface j1_pos_{"j1", hardware_interface::HW_IF_POSITION,
                           &j1_pos_val_};
  CommandInterface j1_vel_{"j1", hardware_interface::HW_IF_VELOCITY,
                           &j1_vel_val_};
  CommandInterface j1_eff_{"j1", hardware_interface::HW_IF_EFFORT,
                           &j1_eff_val_};
  CommandInterface j1_kp_{"j1", HW_IF_GAIN_KP, &j1_kp_val_};
  CommandInterface j1_kd_{"j1", HW_IF_GAIN_KD, &j1_kd_val_};

  CommandInterface j2_pos_{"j2", hardware_interface::HW_IF_POSITION,
                           &j2_pos_val_};
  CommandInterface j2_vel_{"j2", hardware_interface::HW_IF_VELOCITY,
                           &j2_vel_val_};
  CommandInterface j2_eff_{"j2", hardware_interface::HW_IF_EFFORT,
                           &j2_eff_val_};
  CommandInterface j2_kp_{"j2", HW_IF_GAIN_KP, &j2_kp_val_};
  CommandInterface j2_kd_{"j2", HW_IF_GAIN_KD, &j2_kd_val_};

  static StateInterface::SharedPtr make_state_interface(
      const std::string& prefix, const std::string& name,
      const std::string& data_type = "double") {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    info.data_type = data_type;
    return std::make_shared<StateInterface>(
        hardware_interface::InterfaceDescription(prefix, info));
  }
};

// ---------------------------------------------------------------------------

TEST_F(MotkinForwardCommandControllerTest, JointsParamMissing) {
  // Init with empty joints (default) — configure must fail
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "motkin_forward_command_controller";
  params.robot_description = "";
  params.controller_manager_update_rate = 100;
  params.node_namespace = "";
  params.node_options = controller_->define_custom_node_options();
  ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(controller_));
}

TEST_F(MotkinForwardCommandControllerTest, ConfigureSuccess) {
  SetUpController();
  EXPECT_TRUE(configure_succeeds(controller_));
}

TEST_F(MotkinForwardCommandControllerTest, ActivateSuccess) {
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  EXPECT_TRUE(activate_succeeds(controller_));
}

TEST_F(MotkinForwardCommandControllerTest, ActivateWrongJointFails) {
  // Controller configured for j1/j_wrong, but hardware only exposes j1/j2.
  // on_activate returns ERROR → lifecycle transitions to
  // unconfigured/finalized, so activate_succeeds throws rather than returning
  // false.
  SetUpController({"j1", "j_wrong"});
  ASSERT_TRUE(configure_succeeds(controller_));
  EXPECT_THROW(activate_succeeds(controller_), std::runtime_error);
}

TEST_F(MotkinForwardCommandControllerTest, CommandForwardedCorrectly) {
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  // data layout: [pos×2 | vel×2 | eff×2 | kp×2 | kd×2]
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0,  2.0,   // positions  j1, j2
              0.1,  0.2,   // velocities j1, j2
              10.0, 20.0,  // efforts    j1, j2
              5.0,  6.0,   // gains_kp   j1, j2
              0.5,  0.6};  // gains_kd   j1, j2

  controller_->rt_command_.set(cmd);
  EXPECT_EQ(
      controller_->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(j1_pos_val_, 1.0);
  EXPECT_DOUBLE_EQ(j2_pos_val_, 2.0);
  EXPECT_DOUBLE_EQ(j1_vel_val_, 0.1);
  EXPECT_DOUBLE_EQ(j2_vel_val_, 0.2);
  EXPECT_DOUBLE_EQ(j1_eff_val_, 10.0);
  EXPECT_DOUBLE_EQ(j2_eff_val_, 20.0);
  EXPECT_DOUBLE_EQ(j1_kp_val_, 5.0);
  EXPECT_DOUBLE_EQ(j2_kp_val_, 6.0);
  EXPECT_DOUBLE_EQ(j1_kd_val_, 0.5);
  EXPECT_DOUBLE_EQ(j2_kd_val_, 0.6);
}

TEST_F(MotkinForwardCommandControllerTest, NaNSkipped) {
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  j1_pos_val_ = 99.0;
  j2_pos_val_ = 99.0;
  j1_kp_val_ = 99.0;
  j2_kp_val_ = 99.0;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  std_msgs::msg::Float64MultiArray cmd;
  // pos j1=NaN (untouched), j2=3.0; kp j1=7.0, j2=8.0; vel/eff/kd zeros
  cmd.data = {nan, 3.0,   // positions
              0.0, 0.0,   // velocities
              0.0, 0.0,   // efforts
              7.0, 8.0,   // gains_kp
              0.0, 0.0};  // gains_kd

  controller_->rt_command_.set(cmd);
  controller_->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));

  EXPECT_DOUBLE_EQ(j1_pos_val_, 99.0);  // NaN → untouched
  EXPECT_DOUBLE_EQ(j2_pos_val_, 3.0);
  EXPECT_DOUBLE_EQ(j1_kp_val_, 7.0);
  EXPECT_DOUBLE_EQ(j2_kp_val_, 8.0);
}

TEST_F(MotkinForwardCommandControllerTest, WrongSizeIgnored) {
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  j1_pos_val_ = 55.0;
  j2_pos_val_ = 66.0;

  // Only 4 values instead of 10 — entire message must be ignored
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 2.0, 3.0, 4.0};

  controller_->rt_command_.set(cmd);
  controller_->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));

  EXPECT_DOUBLE_EQ(j1_pos_val_, 55.0);  // untouched
  EXPECT_DOUBLE_EQ(j2_pos_val_, 66.0);  // untouched
}

TEST_F(MotkinForwardCommandControllerTest, DeactivateClearsInterfaces) {
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));

  EXPECT_TRUE(controller_->pos_interfaces_.empty());
  EXPECT_TRUE(controller_->vel_interfaces_.empty());
  EXPECT_TRUE(controller_->eff_interfaces_.empty());
  EXPECT_TRUE(controller_->kp_interfaces_.empty());
  EXPECT_TRUE(controller_->kd_interfaces_.empty());
}

TEST_F(MotkinForwardCommandControllerTest, BoardStatePublished) {
  // Hardware exports, in this order: a passive joint, j2 without gains, j1
  // with all the joint interfaces, and the board GPIO.
  std::vector<StateInterface::SharedPtr> ifs;
  auto add = [&](const std::string& prefix, const std::string& name,
                 const std::string& data_type = "double") {
    ifs.push_back(make_state_interface(prefix, name, data_type));
    return ifs.back();
  };
  auto p_pos = add("passive", hardware_interface::HW_IF_POSITION);
  auto j2_pos = add("j2", hardware_interface::HW_IF_POSITION);
  auto j2_vel = add("j2", hardware_interface::HW_IF_VELOCITY);
  auto j2_eff = add("j2", hardware_interface::HW_IF_EFFORT);
  auto j1_pos = add("j1", hardware_interface::HW_IF_POSITION);
  auto j1_vel = add("j1", hardware_interface::HW_IF_VELOCITY);
  auto j1_eff = add("j1", hardware_interface::HW_IF_EFFORT);
  auto j1_kp = add("j1", HW_IF_GAIN_KP);
  auto j1_kd = add("j1", HW_IF_GAIN_KD);
  auto clock = add("motkin_board", "clock", "uint32");
  auto index = add("motkin_board", "latest_command_index", "uint32");
  auto flags = add("motkin_board", "flags", "uint8");

  ASSERT_TRUE(p_pos->set_value(-1.0));
  ASSERT_TRUE(j2_pos->set_value(2.0));
  ASSERT_TRUE(j2_vel->set_value(0.2));
  ASSERT_TRUE(j2_eff->set_value(20.0));
  ASSERT_TRUE(j1_pos->set_value(1.0));
  ASSERT_TRUE(j1_vel->set_value(0.1));
  ASSERT_TRUE(j1_eff->set_value(10.0));
  ASSERT_TRUE(j1_kp->set_value(5.0));
  ASSERT_TRUE(j1_kd->set_value(0.5));
  ASSERT_TRUE(clock->set_value(uint32_t{123456}));
  ASSERT_TRUE(index->set_value(uint32_t{42}));
  ASSERT_TRUE(flags->set_value(uint8_t{3}));

  SetUpController({"j1", "j2"}, {ifs.begin(), ifs.end()});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  EXPECT_STREQ(controller_->board_state_publisher_->get_topic_name(),
               "/motkin_forward_command_controller/board_state");

  const rclcpp::Time stamp(7, 0);
  controller_->update(stamp, rclcpp::Duration::from_seconds(0.01));

  const auto& msg = controller_->board_state_msg_;
  EXPECT_EQ(msg.header.stamp.sec, 7);
  // Controlled joints first, then the others.
  EXPECT_THAT(msg.name, ::testing::ElementsAre("j1", "j2", "passive"));
  EXPECT_THAT(msg.position, ::testing::ElementsAre(1.0, 2.0, -1.0));
  EXPECT_DOUBLE_EQ(msg.velocity[0], 0.1);
  EXPECT_DOUBLE_EQ(msg.velocity[1], 0.2);
  EXPECT_TRUE(std::isnan(msg.velocity[2]));
  EXPECT_DOUBLE_EQ(msg.effort[0], 10.0);
  EXPECT_DOUBLE_EQ(msg.effort[1], 20.0);
  EXPECT_DOUBLE_EQ(msg.gain_kp[0], 5.0);
  EXPECT_DOUBLE_EQ(msg.gain_kd[0], 0.5);
  for (std::size_t i : {1, 2}) {
    EXPECT_TRUE(std::isnan(msg.gain_kp[i]));
    EXPECT_TRUE(std::isnan(msg.gain_kd[i]));
  }
  EXPECT_EQ(msg.clock, 123456u);
  EXPECT_EQ(msg.latest_command_index, 42u);
  EXPECT_EQ(msg.flags, 3u);

  // Values are refreshed at each update.
  ASSERT_TRUE(j1_pos->set_value(1.5));
  ASSERT_TRUE(flags->set_value(uint8_t{0}));
  controller_->update(stamp, rclcpp::Duration::from_seconds(0.01));
  EXPECT_DOUBLE_EQ(msg.position[0], 1.5);
  EXPECT_EQ(msg.flags, 0u);
}
