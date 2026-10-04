#!/usr/bin/env python3
# Copyright 2026 LAAS-CNRS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Endpoint force-to-velocity controller for the five-bar testbed.

Implements qdot = J^T f_c as a plain ROS 2 node that talks to the running
odri_forward_command_controller through its ``~/commands`` topic
(std_msgs/Float64MultiArray, see odri_forward_command_controller/README.md
for the wire format) -- no ros2_control controller plugin is involved here.

Inputs
------
- ``/joint_states`` (sensor_msgs/JointState): used to read the current
  ``motor_1``/``motor_2`` positions (theta1, theta2).
- ``~/contact_force`` (geometry_msgs/WrenchStamped): the contact force
  f_c = (fx, fy) applied at the five-bar closing point, expressed in the
  (x, y) plane of ``case`` (wrench.force.x / wrench.force.y; the
  out-of-plane component and torques are ignored).

Output
------
- ``<forward_command_controller_name>/commands``
  (std_msgs/Float64MultiArray): velocity command plus an analytical
  gravity-compensation feed-forward torque on the effort field (see
  ``five_bar_kinematics.gravity_torque``); position left as NaN (skipped
  by the controller, so the hardware value is left untouched), gain_kp/
  gain_kd set from parameters. Effort is always sent as an explicit
  number (never NaN): relying on NaN-skip for the torque-relevant fields
  means whatever gain was last active (e.g. the controller's own
  initial_command) stays in effect, which is *not* the same as
  commanding zero -- see the effort field of every published message
  here.
"""

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from odri_five_bar_force_velocity_py.five_bar_kinematics import (
    FiveBarGeometry,
    OutOfWorkspaceError,
    SingularConfigurationError,
    gravity_torque,
    joint_velocity_from_contact_force,
)


class FiveBarForceVelocityNode(Node):
    def __init__(self):
        super().__init__("odri_five_bar_force_velocity_node")

        self.declare_parameter("motor_left_joint", "motor_1")
        self.declare_parameter("motor_right_joint", "motor_2")
        self.declare_parameter(
            "forward_command_controller_name", "odri_forward_command_controller"
        )
        self.declare_parameter("contact_force_topic", "~/contact_force")
        self.declare_parameter("gain_kd", 0.05)
        self.declare_parameter("max_joint_velocity", 2.0)  # rad/s, safety clamp
        self.declare_parameter("control_rate", 100.0)  # Hz

        # Default off: the MOTKINBENCH five-bar moves in a horizontal plane, so
        # gravity exerts no in-plane torque. See five_bar_kinematics.gravity_torque.
        self.declare_parameter("gravity_compensation_enabled", False)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("coupler_mass_left", 0.00836663)
        self.declare_parameter("coupler_mass_right", 0.00968954)

        self.declare_parameter("geometry.a_x", -0.0421006)
        self.declare_parameter("geometry.a_z", 0.0302628)
        self.declare_parameter("geometry.b_x", 0.0578994)
        self.declare_parameter("geometry.b_z", 0.0302628)
        self.declare_parameter("geometry.l1", 0.06)
        self.declare_parameter("geometry.l2", 0.1)
        self.declare_parameter("geometry.phi1", 1.5707963267948966)
        self.declare_parameter("geometry.phi2", 1.5707963267948966)

        self._left_joint = self.get_parameter("motor_left_joint").value
        self._right_joint = self.get_parameter("motor_right_joint").value
        fcc_name = self.get_parameter("forward_command_controller_name").value
        self._max_vel = float(self.get_parameter("max_joint_velocity").value)
        self._gain_kd = float(self.get_parameter("gain_kd").value)

        self._gravity_compensation_enabled = bool(
            self.get_parameter("gravity_compensation_enabled").value
        )
        self._gravity = float(self.get_parameter("gravity").value)
        self._coupler_mass_left = float(self.get_parameter("coupler_mass_left").value)
        self._coupler_mass_right = float(self.get_parameter("coupler_mass_right").value)

        self._geom = FiveBarGeometry(
            a_x=self.get_parameter("geometry.a_x").value,
            a_z=self.get_parameter("geometry.a_z").value,
            b_x=self.get_parameter("geometry.b_x").value,
            b_z=self.get_parameter("geometry.b_z").value,
            l1=self.get_parameter("geometry.l1").value,
            l2=self.get_parameter("geometry.l2").value,
            phi1=self.get_parameter("geometry.phi1").value,
            phi2=self.get_parameter("geometry.phi2").value,
        )

        self._theta1 = None
        self._theta2 = None
        self._fx = 0.0
        self._fy = 0.0

        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        contact_force_topic = self.get_parameter("contact_force_topic").value
        self._wrench_sub = self.create_subscription(
            WrenchStamped, contact_force_topic, self._on_contact_force, 10
        )
        self._commands_pub = self.create_publisher(
            Float64MultiArray, f"/{fcc_name}/commands", 10
        )

        control_rate = float(self.get_parameter("control_rate").value)
        self._timer = self.create_timer(1.0 / control_rate, self._control_step)

        self.get_logger().info(
            f"odri_five_bar_force_velocity_node: qdot = J^T f_c, publishing "
            f"to /{fcc_name}/commands from /joint_states and "
            f"{contact_force_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            self._theta1 = msg.position[msg.name.index(self._left_joint)]
            self._theta2 = msg.position[msg.name.index(self._right_joint)]
        except ValueError:
            # joint_states message not (yet) containing our joints
            pass

    def _on_contact_force(self, msg: WrenchStamped) -> None:
        self._fx = msg.wrench.force.x
        self._fy = msg.wrench.force.y

    def _control_step(self) -> None:
        if self._theta1 is None or self._theta2 is None:
            return

        try:
            qdot1, qdot2 = joint_velocity_from_contact_force(
                self._theta1, self._theta2, self._fx, self._fy, self._geom
            )
        except (OutOfWorkspaceError, SingularConfigurationError) as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=1.0)
            return

        qdot1 = max(-self._max_vel, min(self._max_vel, qdot1))
        qdot2 = max(-self._max_vel, min(self._max_vel, qdot2))

        if self._gravity_compensation_enabled:
            tau_g1, tau_g2 = gravity_torque(
                self._theta1,
                self._theta2,
                self._geom,
                self._coupler_mass_left,
                self._coupler_mass_right,
                self._gravity,
            )
        else:
            tau_g1, tau_g2 = 0.0, 0.0

        nan = float("nan")
        msg = Float64MultiArray()
        # Layout: [pos x2 | vel x2 | eff x2 | gain_kp x2 | gain_kd x2]
        # eff is always an explicit number (never NaN): with NaN-skip, the
        # last active gain/effort stays in effect, which is not the same
        # as commanding zero torque.
        msg.data = [
            nan,
            nan,
            qdot1,
            qdot2,
            tau_g1,
            tau_g2,
            0.0,
            0.0,
            self._gain_kd,
            self._gain_kd,
        ]
        self._commands_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FiveBarForceVelocityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
