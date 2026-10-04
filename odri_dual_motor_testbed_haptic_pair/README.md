# odri_dual_motor_testbed_haptic_pair

Simulates two `odri_dual_motor_testbed` five-bar robots as a haptic pair in
one Gazebo world: a **leader** that receives an external contact force (its
haptic-sensor input) and a **follower** that reproduces it.

## Architecture

The launch file starts one world with
`odri_dual_motor_testbed_gazebo`'s `gz_world.launch.py`, then includes
`odri_dual_motor_testbed_gazebo`'s `robot_spawn.launch.py` twice, with
`namespace:=leader` and `namespace:=follower`. Each robot gets its own
`controller_manager` (`/leader/controller_manager`,
`/follower/controller_manager`), which loads the *existing*
`odri_five_bar_force_velocity_controller` (from
`odri_five_bar_force_velocity_controller`). That controller already
implements the admittance law `qdot = J^T f_c`, which turns a sensed contact
force into motion for this mechanism.

The only new code is `contact_force_relay`: it republishes whatever
`geometry_msgs/WrenchStamped` the leader receives on
`/leader/odri_five_bar_force_velocity_controller/contact_force` onto
`/follower/odri_five_bar_force_velocity_controller/contact_force`. Since the
follower runs the identical controller, it reproduces the same force -- and,
by construction of the shared admittance law, the same motion -- with no new
control algorithm.

```
[external force] -> leader/contact_force -> leader controller -> leader motion
                            |
                     contact_force_relay
                            v
                     follower/contact_force -> follower controller -> follower motion
```

## Running

```bash
ros2 launch odri_dual_motor_testbed_haptic_pair haptic_pair.launch.py
```

The Gazebo GUI uses `config/haptic_pair.config` by default, with the camera
set to show both robots. Use `gui_config:=<file>` to load another file, or
`gui:=false` to run without a GUI.

Drive the leader (from a script, joystick bridge, or Gazebo's own
apply-force tool bridged to ROS):

```bash
ros2 topic pub /leader/odri_five_bar_force_velocity_controller/contact_force \
  geometry_msgs/msg/WrenchStamped "{wrench: {force: {x: 1.0}}}"
```

Compare `/leader/joint_states` and `/follower/joint_states` to see the
follower reproduce the leader's motion.

## Implementation notes

- **Params YAML must use the `/**` wildcard once namespaced.** A bare
  `controller_manager:` top-level key in the params YAML only matches a
  node at `/controller_manager` (root namespace). Once the node is
  namespaced to `/leader` or `/follower`, that key silently stops
  matching. Controllers then fail to load with `The 'type' param was not
  defined for '<controller>'`, and nothing says the namespace is the cause.
  The fix is to key everything under `/**:`, as
  `haptic_pair_controllers.yaml` does.

- **Gazebo entity names must be unique.** `robot_spawn.launch.py` names
  each model `<namespace>_<robot_model>` (passed to `ros_gz_sim create`
  with `-name`). With two robots spawned at runtime under the same name,
  only the first one comes up.
