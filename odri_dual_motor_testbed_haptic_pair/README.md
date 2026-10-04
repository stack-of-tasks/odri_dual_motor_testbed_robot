# odri_dual_motor_testbed_haptic_pair

Simulates two `odri_dual_motor_testbed` five-bar robots as a haptic pair in
one Gazebo world: a **leader** that receives an external contact force and a
**follower** that reproduces it.

Quick start:

```bash
ros2 launch odri_dual_motor_testbed_haptic_pair haptic_pair.launch.py
```

The full documentation is in `doc/`, built with rosdoc2.
