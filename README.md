# ODRI dual motor testbed robot

`ros2_control` software for the ODRI dual motor testbed: a base with two
motors, driven by a Raspberry Pi Pico dual DRV8316C board over USB, on which
either a planar five-bar linkage (`fivebar_2dof`) or two flywheels
(`dual_flywheel`) are mounted.

The same robot description, controllers and commands are used on the real
robot and in Gazebo Harmonic.

## Packages

| Package | Content |
|---|---|
| `odri_dual_motor_testbed_robot` | Meta-package and entry point of the documentation |
| `odri_dual_motor_testbed_description` | URDF/xacro models, meshes, `ros2_control` declarations, RViz display |
| `odri_dual_motor_testbed_bringup` | Launch files and controller configuration for the real robot |
| `odri_dual_motor_testbed_hardware` | `FiveBarSystem` hardware plugin: passive joints of the five-bar computed from the motor encoders |
| `odri_dual_motor_testbed_gazebo` | Gazebo simulation, and `FiveBarClosurePlugin`, which closes the five-bar loop |
| `odri_dual_motor_testbed_controllers/odri_forward_command_controller` | Forwards position, velocity, effort, `gain_kp` and `gain_kd` commands to the motors from one topic |
| `odri_dual_motor_testbed_controllers/odri_five_bar_force_velocity_controller` | Moves the five-bar end point in response to a contact force, q̇ = Jᵀ f |
| `odri_dual_motor_testbed_controllers/odri_five_bar_force_velocity_py` | The same control law as a Python node |
| `odri_dual_motor_testbed_haptic_pair` | Two simulated five-bars: a follower reproduces the force applied to a leader |

`five_bar_mgd_spec.md` derives the direct geometric model of the five-bar used
by the hardware plugin, the Gazebo plugin and the controllers.

## Installation

In a ROS 2 Jazzy workspace:

```bash
mkdir -p odri_dual_motor_testbed_ws/src
cd odri_dual_motor_testbed_ws/src
git clone https://github.com/stack-of-tasks/odri_dual_motor_testbed_robot.git
cd ..
vcs import --skip-existing src < src/odri_dual_motor_testbed_robot/odri_dual_motor_testbed_robot.repos
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source ./install/setup.bash
```

`odri_dual_motor_testbed_robot.repos` lists the repositories the workspace
needs, besides this one:

- [`odri_gz_ros2_control`](https://github.com/stack-of-tasks/odri_gz_ros2_control):
  `ros2_control` hardware interface for Gazebo, needed for simulation;
- [`pico_dual_drv8316c_ros2_hardware_interface`](https://github.com/Gepetto/pico_dual_drv8316c_ros2_hardware_interface):
  hardware interface of the motor board, needed for the real robot.

The file also lists this repository, so `--skip-existing` keeps the clone made
above untouched.

## Usage

Display a model in RViz, with a slider per joint:

```bash
ros2 launch odri_dual_motor_testbed_description show.launch.py robot_model:=fivebar_2dof
```

Simulate it in Gazebo (add `gui:=false` for a headless run):

```bash
ros2 launch odri_dual_motor_testbed_gazebo odri_dual_motor_testbed_gazebo.launch.py robot_model:=fivebar_2dof
```

Start the real robot, with the board plugged in:

```bash
ros2 launch odri_dual_motor_testbed_bringup odri_dual_motor_testbed_ctrl.launch.py robot_model:=fivebar_2dof
```

In simulation or on the real robot, `odri_forward_command_controller` is
started with every gain at 0. Hold both motors at 0 with a PD:

```bash
ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  5.0, 5.0,  0.1, 0.1]}"
#          position    velocity    effort     gain_kp     gain_kd
```

Replace `fivebar_2dof` with `dual_flywheel` for the flywheel model.

## Documentation

Each package has a `doc/` directory built with
[rosdoc2](https://github.com/ros-infrastructure/rosdoc2). The entry point is
the documentation of the `odri_dual_motor_testbed_robot` meta-package.

The documentation is built by `colcon build`, like the rest of the
workspace, when `rosdoc2` is available.

Each package installs its HTML documentation in
`install/<package>/share/<package>/doc`. Start with the meta-package:

```bash
xdg-open install/odri_dual_motor_testbed_robot/share/odri_dual_motor_testbed_robot/doc/index.html
```

To build the documentation of a package by hand, without colcon, run
`rosdoc2 build --package-path <package>` from the directory that contains
it. The result goes to `docs_output/<package>/index.html`.

## License

Apache License 2.0.
