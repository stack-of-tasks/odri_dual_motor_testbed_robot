odri_forward_command_controller
===============================

A ``ros2_control`` controller that forwards position, velocity, effort,
``gain_kp`` and ``gain_kd`` commands to a set of ODRI joints through a single
topic.

It follows the design of ``forward_command_controller`` from
``ros2_controllers`` and uses ``std_msgs/msg/Float64MultiArray`` as command
message. It is the controller started by ``odri_dual_motor_testbed_bringup``
on the real robot and by ``odri_dual_motor_testbed_gazebo`` in simulation,
on ``motor_1`` and ``motor_2``.

The ODRI motor boards turn these five commands into a torque:

.. math::

   \tau = \tau_{cmd} + K_p (q_{cmd} - q) + K_d (\dot q_{cmd} - \dot q)

Quick start, with the testbed running:

.. code-block:: bash

   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  5.0, 5.0,  0.1, 0.1]}"

.. toctree::
   :maxdepth: 2
   :caption: Contents

   command_interface
   configuration
