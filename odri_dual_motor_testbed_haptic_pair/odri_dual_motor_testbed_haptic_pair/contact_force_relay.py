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
"""Relay the leader's sensed contact force onto the follower's.

Both robots of the haptic pair run the same
odri_five_bar_force_velocity_controller, which turns a contact force into
motion (qdot = J^T f_c). The leader's contact force topic is the haptic
input (whatever pushes on it -- a user, a Gazebo "apply force" tool, a
script). This node republishes that same geometry_msgs/WrenchStamped onto
the follower's contact force topic, unchanged: since both controllers
implement the identical admittance law, the follower reproduces the force
-- and therefore the motion -- felt by the leader, with no new control
algorithm involved.
"""

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node


class ContactForceRelay(Node):
    def __init__(self):
        super().__init__("contact_force_relay")

        self.declare_parameter(
            "input_topic",
            "/leader/odri_five_bar_force_velocity_controller/contact_force",
        )
        self.declare_parameter(
            "output_topic",
            "/follower/odri_five_bar_force_velocity_controller/contact_force",
        )

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._pub = self.create_publisher(WrenchStamped, output_topic, 10)
        self._sub = self.create_subscription(
            WrenchStamped, input_topic, self._pub.publish, 10
        )

        self.get_logger().info(f"relaying {input_topic} -> {output_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = ContactForceRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
