#!/usr/bin/env python3
"""Send one command to the MOTKIN forward command controller.

Equivalent to:
    ros2 topic pub --once /motkin_forward_command_controller/commands \\
      std_msgs/msg/Float64MultiArray \\
      "{data: [-0.5, 0.3, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.05, 0.05]}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

positions = [-0.5, 0.3]  # rad
velocities = [0.0, 0.0]  # rad/s
efforts = [0.0, 0.0]  # N.m
gains_kp = [1.0, 1.0]
gains_kd = [0.05, 0.05]


def main():
    rclpy.init()
    node = Node("motkin_send_command")
    pub = node.create_publisher(
        Float64MultiArray, "/motkin_forward_command_controller/commands", 10
    )

    # Wait for the controller to subscribe, otherwise the message is lost.
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = Float64MultiArray()
    msg.data = positions + velocities + efforts + gains_kp + gains_kd
    pub.publish(msg)
    node.get_logger().info(f"published {msg.data.tolist()}")

    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
