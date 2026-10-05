motkin_five_bar_force_velocity_py
===============================

A Python ROS 2 node that moves the five-bar of the MOTKIN dual motor testbed in
response to a force applied at its end point P, with the control law

.. math::

   \dot q = J^T f_c

It is the topic-based counterpart of ``motkin_five_bar_force_velocity_controller``:
same control law and same analytical Jacobian, but instead of being a
``ros2_control`` plugin, the node reads ``/joint_states`` and publishes on the
command topic of a running ``motkin_forward_command_controller``. It can be
started and stopped without touching the ``controller_manager``.

Quick start, with the testbed running (real robot or Gazebo):

.. code-block:: bash

   ros2 launch motkin_five_bar_force_velocity_py force_velocity.launch.py

   ros2 topic pub -r 20 /motkin_five_bar_force_velocity_node/contact_force \
     geometry_msgs/msg/WrenchStamped "{wrench: {force: {x: 1.0, y: 0.0, z: 0.0}}}"

.. toctree::
   :maxdepth: 2
   :caption: Contents

   node
   running
