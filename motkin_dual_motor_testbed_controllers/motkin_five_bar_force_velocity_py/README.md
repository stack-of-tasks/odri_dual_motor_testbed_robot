# motkin_five_bar_force_velocity_py

A plain Python ROS 2 node implementing the endpoint force-to-velocity
control law **qdot = J<sup>T</sup> f_c** for the five-bar mechanism of the
`motkin_dual_motor_testbed`, driven entirely through the
[`motkin_forward_command_controller`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/tree/main/motkin_dual_motor_testbed_controllers/motkin_forward_command_controller)
topic interface (`~/commands`, `std_msgs/msg/Float64MultiArray`) — no
`ros2_control` controller plugin here. Its `ros2_control`-native counterpart
is [`motkin_five_bar_force_velocity_controller`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/tree/main/motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_controller).

## Mechanism and Jacobian

See
[`five_bar_mgd_spec.md`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/blob/main/five_bar_mgd_spec.md)
for the direct geometric model of the five-bar this is built on:
`motor_1`/`motor_2` (theta1, theta2) each drive a crank of length `L1` down to
an elbow, itself connected to the common closing point `P` (the mechanism's
end point) through a coupler of length `L2`. `P` is obtained in closed form
as the intersection of two circles, without any iterative solver.

[`motkin_five_bar_force_velocity_py/five_bar_kinematics.py`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/blob/main/motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_py/motkin_five_bar_force_velocity_py/five_bar_kinematics.py)
reimplements that direct model and derives its 2x2 velocity Jacobian `J`
(`Pdot = J @ thetadot`) **analytically**, by implicit differentiation of the
two loop-closure constraints `|P - E_L|^2 = L2^2` and `|P - E_R|^2 = L2^2`.
No Pinocchio (or any other rigid-body/kinematics library) is used — `J` is a
direct algebraic function of `(theta1, theta2)` and of the constant
geometric parameters read off the URDF (`A`, `B`, `L1`, `L2`, `phi1`,
`phi2`), matching exactly the parameters in `five_bar_mgd_spec.md`.

The node then commands

```
qdot = J^T f_c
```

with `f_c = (fx, fy)` the contact force applied at `P`, expressed in the
`(x, y)` plane of `case`. Using `J^T` directly (instead of `J^-T` or a
proper admittance model) is a deliberate simplification for this teaching
example: it needs no matrix inversion, has no singularity in the force path,
and always drives the joints in the direction that is power-conjugate to the
applied Cartesian force.

## Gravity compensation

> **Disabled by default on the MOTKINBENCH five-bar.** That mechanism moves
> in the `(x, y)` plane of `case` with both motor axes vertical, so gravity is
> perpendicular to the plane and exerts no torque on the motors. The coupler
> centres of mass no longer sit at the elbows either (`arm_l2`'s `<inertial>`
> origin is `(0.0487, 0, -0.002)`). The derivation below holds only for a
> vertically-mounted five-bar whose coupler CoMs are at the elbows; re-deriving
> it for a vertical remount means redoing both assumptions.

The `effort` slice of every published command is an **analytical
gravity-compensation feed-forward torque**, not a fixed zero — see
`gravity_torque()` in
[`five_bar_kinematics.py`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/blob/main/motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_py/motkin_five_bar_force_velocity_py/five_bar_kinematics.py).
In short: the coupler links `arm_l2`/`arm_r2` have their center of mass at the
elbow per the URDF's own inertial data, so their world height is exactly
`E_L(theta1)` / `E_R(theta2)`, and the feed-forward torque is `dV/dtheta`
for `V = g * (m_left * E_Lz(theta1) + m_right * E_Rz(theta2))`:

```
tau_g1 = coupler_mass_left  * gravity * (-L1 * cos(phi1 - theta1))
tau_g2 = coupler_mass_right * gravity * (-L1 * cos(phi2 - theta2))
```

The crank's own mass (rotor + arm) is *not* compensated: its combined
center of mass sits within ~0.25 mm of its own motor axis (verified by
composing the actual URDF joint transforms), contributing under ~1% of the
coupler term — not worth a second fragile fitted term.

This alone gives static equilibrium (no `gain_kp` needed to hold position
against gravity), but any unmodeled effect (the crank's own ~1% residual,
motor cogging, friction) will still leave a slow residual drift; pair it
with a small nonzero `gain_kp` if you need zero steady-state error. Set
`gravity_compensation_enabled:=false` to publish a literal `effort = 0.0`
instead.

## Wiring

- Subscribes to `/joint_states` (`sensor_msgs/msg/JointState`) for the
  `motor_1`/`motor_2` positions (theta1, theta2) — published by
  `joint_state_broadcaster`.
- Subscribes to `~/contact_force`
  (`geometry_msgs/msg/WrenchStamped`, `wrench.force.x`/`wrench.force.y` used
  as `fx`/`fy`, expressed in the `case` `(x, y)` plane).
- Publishes to `/<forward_command_controller_name>/commands`
  (`std_msgs/msg/Float64MultiArray`), velocity + gravity feed-forward:
  `[NaN, NaN, qdot1, qdot2, tau_g1, tau_g2, 0.0, 0.0, gain_kd, gain_kd]`
  (position left as NaN so `motkin_forward_command_controller` leaves the
  hardware value untouched; `effort` and `gain_kp` are always sent as
  explicit numbers, never NaN — relying on NaN-skip for torque-relevant
  fields means whatever gain was last active stays in effect, which is
  *not* the same as commanding zero; `gain_kp` explicitly 0 so `gain_kd`
  is what turns `qdot` into torque on top of gravity compensation — see
  `ros2_hardware_interface_motkin`).

If the current configuration is outside the workspace or the Jacobian is
singular, the node logs a throttled warning and skips that control cycle
(no command published) rather than publishing a NaN/huge velocity.

## Running

Assuming the testbed is already brought up with `motkin_forward_command_controller` active (see
`motkin_dual_motor_testbed_bringup`/`motkin_dual_motor_testbed_gazebo`):

```bash
ros2 launch motkin_five_bar_force_velocity_py force_velocity.launch.py

ros2 topic pub -r 20 /motkin_five_bar_force_velocity_node/contact_force \
  geometry_msgs/msg/WrenchStamped "{wrench: {force: {x: 1.0, y: 0.0, z: 0.0}}}"
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `motor_left_joint` | `motor_1` | Left actuated joint name (theta1) |
| `motor_right_joint` | `motor_2` | Right actuated joint name (theta2) |
| `forward_command_controller_name` | `motkin_forward_command_controller` | Controller to publish `~/commands` to |
| `contact_force_topic` | `~/contact_force` | `WrenchStamped` input topic |
| `gain_kd` | `0.05` | Velocity gain sent with each command |
| `max_joint_velocity` | `2.0` | Safety clamp on `qdot` (rad/s) |
| `control_rate` | `100.0` | Control loop rate (Hz) |
| `gravity_compensation_enabled` | `false` | If false, publish `effort = 0.0` instead of the gravity feed-forward |
| `gravity` | `9.81` | Gravitational acceleration (m/s^2) |
| `coupler_mass_left` | `0.00836663` | Mass (kg) of `arm_l2`, used for the `motor_1` gravity feed-forward |
| `coupler_mass_right` | `0.00968954` | Mass (kg) of `arm_r2`, used for the `motor_2` gravity feed-forward |
| `geometry.*` | see `five_bar_mgd_spec.md` | Five-bar geometric parameters, read off the URDF |

## Building

```bash
cd <workspace>
colcon build --packages-select motkin_five_bar_force_velocity_py
source install/setup.bash
```
