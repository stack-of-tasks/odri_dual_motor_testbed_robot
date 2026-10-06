# motkin_dual_motor_testbed_haptic_pair

Runs two `motkin_dual_motor_testbed` five-bar robots as a haptic pair, either
simulated in one Gazebo world or as two real kits on the same computer: a **leader** that receives an external contact force and a
**follower** that reproduces it.

Quick start:

```bash
# Simulation (Gazebo)
ros2 launch motkin_dual_motor_testbed_haptic_pair haptic_pair_gazebo.launch.py

# Two real kits plugged into the same computer
ros2 launch motkin_dual_motor_testbed_haptic_pair haptic_pair.launch.py \
  leader_serial_port:=/dev/serial/by-id/usb-...-leader \
  follower_serial_port:=/dev/serial/by-id/usb-...-follower
```

The full documentation is in `doc/`, built with rosdoc2.
