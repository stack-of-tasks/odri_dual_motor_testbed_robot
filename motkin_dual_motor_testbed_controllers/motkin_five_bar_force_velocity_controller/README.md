# motkin_five_bar_force_velocity_controller

A `ros2_control` controller plugin implementing an endpoint force-to-velocity
control law, **qdot = J<sup>T</sup> f_c**, for the five-bar mechanism of the
`motkin_dual_motor_testbed`.

This is the `ros2_control` counterpart of
[`motkin_five_bar_force_velocity_py`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/tree/main/motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_py): same
control law, same analytical Jacobian, but running as a controller plugin
loaded by `controller_manager` instead of talking to
`motkin_forward_command_controller` over a topic. It directly claims the
`motor_1`/`motor_2` command interfaces.

## Mechanism and Jacobian

See
[`five_bar_mgd_spec.md`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/blob/main/five_bar_mgd_spec.md)
for the direct geometric model of the five-bar this is built on. In short:
`motor_1`/`motor_2` (theta1, theta2) each drive a crank of length `L1` down to
an elbow, itself connected to the common closing point `P` (the mechanism's
end point) through a coupler of length `L2`. `P` is obtained in closed form
as the intersection of two circles, without any iterative solver.

[`include/motkin_five_bar_force_velocity_controller/five_bar_kinematics.hpp`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/blob/main/motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_controller/include/motkin_five_bar_force_velocity_controller/five_bar_kinematics.hpp)
reimplements that direct model and derives its 2x2 velocity Jacobian `J`
(`Pdot = J * thetadot`) **analytically**, by implicit differentiation of the
two loop-closure constraints `|P - E_L|^2 = L2^2` and `|P - E_R|^2 = L2^2`.
No Pinocchio (or any other rigid-body/kinematics library) is used — `J` is a
direct algebraic function of `(theta1, theta2)` and of the constant
geometric parameters read off the URDF (`A`, `B`, `L1`, `L2`, `phi1`,
`phi2`), matching exactly the parameters in `five_bar_mgd_spec.md`.

The controller then commands

```
qdot = J^T f_c
```

with `f_c = (fx, fy)` the contact force applied at `P`, expressed in the
`(x, y)` plane of `case`. Using `J^T` directly (instead of `J^-T` or a
proper admittance model) is a deliberate simplification for this teaching
example: it needs no matrix inversion, has no singularity in the force path,
and always drives the joints in the direction that is power-conjugate to the
applied Cartesian force.

`test/test_five_bar_kinematics.cpp` cross-checks the analytical Jacobian
against a central finite-difference of the direct geometric model over
several configurations, as a sanity check on the closed-form derivation.

## Gravity compensation

> **Disabled by default on the MOTKINBENCH five-bar.** That mechanism moves
> in the `(x, y)` plane of `case` with both motor axes vertical, so gravity is
> perpendicular to the plane and exerts no torque on the motors. The coupler
> centres of mass no longer sit at the elbows either (`arm_l2`'s `<inertial>`
> origin is `(0.0487, 0, -0.002)`). The derivation below holds only for a
> vertically-mounted five-bar whose coupler CoMs are at the elbows; re-deriving
> it for a vertical remount means redoing both assumptions.

The `effort` command sent every cycle is an **analytical gravity-compensation
feed-forward torque**, not a fixed zero. See
`GravityTorque()` in
[`five_bar_kinematics.hpp`](https://github.com/stack-of-tasks/motkin_dual_motor_testbed_robot/blob/main/motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_controller/include/motkin_five_bar_force_velocity_controller/five_bar_kinematics.hpp)
for the full derivation; in short: the coupler links `arm_l2`/`arm_r2` have their
center of mass at the elbow per the URDF's own `<inertial>` data, so their
world height is exactly `E_L(theta1)` / `E_R(theta2)` (no loop-closure
coupling needed), and the feed-forward torque is `tau_g = dV/dtheta` for
`V = g * (m_left * E_Lz(theta1) + m_right * E_Rz(theta2))`:

```
tau_g1 = coupler_mass_left  * gravity * (-L1 * cos(phi1 - theta1))
tau_g2 = coupler_mass_right * gravity * (-L1 * cos(phi2 - theta2))
```

The crank's own mass (rotor + arm) is *not* compensated: its combined
center of mass sits within ~0.25 mm of its own motor axis (verified by
composing the actual URDF joint transforms), contributing under ~1% of the
coupler term — not worth a second fragile fitted term. `test/test_five_bar_kinematics.cpp`
cross-checks `GravityTorque()` against a finite difference of this potential
energy.

This alone gives static equilibrium (no residual `gain_kp` is needed to hold
position against gravity), but any unmodeled effect — the crank's own ~1%
residual, motor cogging, friction — will still leave a slow residual drift;
pair it with a small nonzero `gain_kp` if you need zero steady-state error.
Set `gravity_compensation_enabled: false` to send a literal `effort = 0.0`
instead (e.g. to reproduce/compare against pure free-drift behavior).

## What it does at each `update()`

1. Reads `motor_1/position` and `motor_2/position` state interfaces
   (theta1, theta2).
2. Reads the latest contact force from the `contact_force` topic
   (`geometry_msgs/msg/WrenchStamped`, `wrench.force.x`/`wrench.force.y`
   used as `fx`/`fy`; expressed in the `case` `(x, y)` plane).
3. Computes `qdot = J^T f_c` with the analytical Jacobian, clamped to
   `max_joint_velocity`.
4. Computes the gravity-compensation feed-forward torque (see above), unless
   `gravity_compensation_enabled` is false.
5. Writes, on both `motor_1` and `motor_2`:
   - `position` = currently measured position (position setpoint held at
     the measured value, so `gain_kp` only acts as an anti-drift term),
   - `velocity` = the corresponding component of `qdot`,
   - `effort` = the corresponding component of the gravity feed-forward
     (or `0.0` if disabled) — always an explicit number, never left
     unset, so "no extra torque beyond gravity compensation" always means
     exactly that,
   - `gain_kp` / `gain_kd` = the `gain_kp` / `gain_kd` parameters (`gain_kd`
     is what actually turns the velocity setpoint into torque; see
     `ros2_hardware_interface_motkin`).

If the current configuration is outside the workspace or the Jacobian is
singular, the controller logs a throttled warning and commands `qdot = 0`
(holds still) rather than propagating a NaN/huge value to the hardware.

## Configuration

```yaml
controller_manager:
  ros__parameters:
    motkin_five_bar_fv:
      type: motkin_five_bar_force_velocity_controller/MotkinFiveBarForceVelocityController

motkin_five_bar_fv:
  ros__parameters:
    motor_left_joint: motor_1
    motor_right_joint: motor_2
    contact_force_topic: contact_force
    gain_kp: 0.0
    gain_kd: 0.05
    max_joint_velocity: 2.0
    gravity_compensation_enabled: false
    gravity: 9.81
    coupler_mass_left: 0.00836663
    coupler_mass_right: 0.00968954
    geometry:
      a_x: -0.0421006
      a_z: 0.0302628
      b_x: 0.0578994
      b_z: 0.0302628
      l1: 0.06
      l2: 0.1
      phi1: 1.5707963267948966
      phi2: 1.5707963267948966
```

The `geometry` block defaults to the values above (read off
`fivebar_2dof.urdf.xacro`/`five_bar_mgd_spec.md`) and only needs overriding
if the mechanical design changes.

## Trying it out

```bash
ros2 control load_controller --set-state active motkin_five_bar_fv

ros2 topic pub -r 20 /motkin_five_bar_fv/contact_force geometry_msgs/msg/WrenchStamped \
  "{wrench: {force: {x: 1.0, y: 0.0, z: 0.0}}}"
```

## Building

```bash
cd <workspace>
colcon build --packages-select motkin_five_bar_force_velocity_controller
source install/setup.bash
```

## Testing

```bash
colcon test --packages-select motkin_five_bar_force_velocity_controller
colcon test-result --verbose
```
