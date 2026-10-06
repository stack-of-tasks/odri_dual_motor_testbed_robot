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
"""
Couple two real five-bar kits: position forward, force (current) feedback.

The real kits have no force sensor, so contact_force_relay (which forwards
a force published on a topic) only makes sense in simulation. Here both
kits run motkin_forward_command_controller and this node drives them
through the PD + feed-forward law of the firmware
(iq = iff + kp * (q_target - q) + kd * (v_target - v)):

- follower: q_target = q_leader, v_target = v_leader, gains follower_kp /
  follower_kd -- the follower tracks the leader moved by hand;
- leader: feedback of the force the follower meets, estimated from the
  follower's measured motor current (the ``effort`` field of its
  joint_states, in A). Both kits have the same kinematics and, once
  coupled, the same joint angles, so the Jacobians of the force -> torque
  mappings cancel and the estimate can be rendered joint by joint:
  iff_leader = -force_feedback_gain * lowpass(i_follower), clamped to
  +/- max_feedback_current. When the follower is blocked, its PD current
  rises and the user feels it on the leader. The follower current also
  contains what moves the follower itself (friction, inertia), so the user
  feels that too, scaled by force_feedback_gain;
- optionally, a PD of the leader towards the follower (leader_kp /
  leader_kd, 0 by default): position-error feedback instead of, or on top
  of, the current feedback.

Safety:

- the position error used in each target is clamped to
  +/- max_position_error, so a large initial mismatch does not make the
  follower jump at full current;
- if the joint states of either kit are missing or older than
  state_timeout, or when the node exits, zero gains and zero current are
  sent to both kits (motors limp).
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class _JointStateCache:
    """Latest positions, velocities and currents of the coupled joints."""

    def __init__(self, joints):
        self.joints = joints
        self.position = None
        self.velocity = None
        self.effort = None
        self.stamp = None

    def update(self, msg, now):
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        n = len(msg.name)
        self.position = [msg.position[i] for i in idx]
        self.velocity = [
            msg.velocity[i] if len(msg.velocity) == n else 0.0 for i in idx
        ]
        self.effort = [msg.effort[i] if len(msg.effort) == n else 0.0 for i in idx]
        self.stamp = now


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class PositionCoupling(Node):
    def __init__(self):
        super().__init__("position_coupling")

        self.declare_parameter("leader_namespace", "leader")
        self.declare_parameter("follower_namespace", "follower")
        self.declare_parameter(
            "controller_name", "motkin_forward_command_controller"
        )
        self.declare_parameter("joints", ["motor_1", "motor_2"])
        self.declare_parameter("follower_kp", 2.0)
        self.declare_parameter("follower_kd", 0.05)
        self.declare_parameter("force_feedback_gain", 0.5)
        self.declare_parameter("force_feedback_cutoff", 10.0)  # Hz
        self.declare_parameter("max_feedback_current", 0.5)  # A
        self.declare_parameter("leader_kp", 0.0)
        self.declare_parameter("leader_kd", 0.0)
        self.declare_parameter("max_position_error", 0.3)  # rad
        self.declare_parameter("state_timeout", 0.1)  # s
        self.declare_parameter("control_rate", 100.0)  # Hz

        leader_ns = self.get_parameter("leader_namespace").value
        follower_ns = self.get_parameter("follower_namespace").value
        controller = self.get_parameter("controller_name").value
        joints = list(self.get_parameter("joints").value)

        self._leader = _JointStateCache(joints)
        self._follower = _JointStateCache(joints)
        self._feedback_current = [0.0] * len(joints)
        self._was_coupled = False

        self.create_subscription(
            JointState,
            f"/{leader_ns}/joint_states",
            lambda msg: self._leader.update(msg, self._now()),
            10,
        )
        self.create_subscription(
            JointState,
            f"/{follower_ns}/joint_states",
            lambda msg: self._follower.update(msg, self._now()),
            10,
        )
        self._leader_pub = self.create_publisher(
            Float64MultiArray, f"/{leader_ns}/{controller}/commands", 10
        )
        self._follower_pub = self.create_publisher(
            Float64MultiArray, f"/{follower_ns}/{controller}/commands", 10
        )

        self._period = 1.0 / self.get_parameter("control_rate").value
        self.create_timer(self._period, self._step)

        p = self.get_parameter
        self.get_logger().info(
            f"coupling /{follower_ns} to /{leader_ns} on {joints} "
            f"(follower kp={p('follower_kp').value} kd={p('follower_kd').value}; "
            f"force feedback gain={p('force_feedback_gain').value}; "
            f"leader kp={p('leader_kp').value} kd={p('leader_kd').value})"
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, cache):
        timeout = self.get_parameter("state_timeout").value
        return cache.stamp is not None and self._now() - cache.stamp <= timeout

    def _command(self, own, other, kp, kd, effort):
        # Layout: [pos x n | vel x n | eff x n | gain_kp x n | gain_kd x n]
        max_err = self.get_parameter("max_position_error").value
        n = len(own.joints)
        pos = [
            q + _clamp(q_ref - q, max_err)
            for q, q_ref in zip(own.position, other.position)
        ]
        msg = Float64MultiArray()
        msg.data = pos + list(other.velocity) + list(effort) + [kp] * n + [kd] * n
        return msg

    def _zero_command(self):
        n = len(self._leader.joints)
        msg = Float64MultiArray()
        # NaN position: keep the last target; velocity, effort and gains
        # explicitly 0 so that no current is commanded.
        msg.data = [math.nan] * n + [0.0] * (4 * n)
        return msg

    def _update_feedback_current(self):
        """Low-pass filter, scale and clamp the follower current."""
        cutoff = self.get_parameter("force_feedback_cutoff").value
        tau = 1.0 / (2.0 * math.pi * cutoff)
        alpha = self._period / (tau + self._period)
        gain = self.get_parameter("force_feedback_gain").value
        i_max = self.get_parameter("max_feedback_current").value
        self._feedback_current = [
            _clamp(f + alpha * (-gain * i - f), i_max)
            for f, i in zip(self._feedback_current, self._follower.effort)
        ]

    def release(self):
        """Send zero gains and zero current to both kits (motors limp)."""
        self._feedback_current = [0.0] * len(self._feedback_current)
        self._leader_pub.publish(self._zero_command())
        self._follower_pub.publish(self._zero_command())

    def _step(self):
        if not (self._fresh(self._leader) and self._fresh(self._follower)):
            if self._was_coupled:
                self.get_logger().warn(
                    "joint states missing or stale, releasing both kits"
                )
                self._was_coupled = False
            self.release()
            return

        if not self._was_coupled:
            self.get_logger().info("both kits alive, coupling engaged")
            self._was_coupled = True

        self._update_feedback_current()
        n = len(self._feedback_current)
        self._follower_pub.publish(
            self._command(
                self._follower,
                self._leader,
                self.get_parameter("follower_kp").value,
                self.get_parameter("follower_kd").value,
                [0.0] * n,
            )
        )
        self._leader_pub.publish(
            self._command(
                self._leader,
                self._follower,
                self.get_parameter("leader_kp").value,
                self.get_parameter("leader_kd").value,
                self._feedback_current,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = PositionCoupling()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
