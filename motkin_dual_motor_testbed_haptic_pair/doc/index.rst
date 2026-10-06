motkin_dual_motor_testbed_haptic_pair
=====================================

Runs two ``motkin_dual_motor_testbed`` five-bar robots as a haptic pair, either
simulated in one Gazebo world or as two real kits on the same computer: a **leader** that receives an external contact force (its
haptic-sensor input) and a **follower** that reproduces it.

.. figure:: haptic_pair_in_gz_harmonic.png
   :width: 100%
   :alt: The leader and follower five-bars in Gazebo Harmonic

   The leader (left) and the follower (right) in Gazebo Harmonic.

Quick start:

.. code-block:: bash

   # Simulation (Gazebo)
   ros2 launch motkin_dual_motor_testbed_haptic_pair haptic_pair_gazebo.launch.py

   # Two real kits plugged into the same computer
   ros2 launch motkin_dual_motor_testbed_haptic_pair haptic_pair.launch.py \
     leader_serial_port:=/dev/serial/by-id/usb-...-leader \
     follower_serial_port:=/dev/serial/by-id/usb-...-follower

.. toctree::
   :maxdepth: 2
   :caption: Contents

   package_structure
   architecture
   launch_files
   implementation_notes
