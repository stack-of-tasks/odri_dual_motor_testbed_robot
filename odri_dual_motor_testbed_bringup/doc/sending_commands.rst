Sending commands
================

``odri_dual_motor_testbed_ctrl.launch.py`` starts
``odri_forward_command_controller``. It sends position, velocity, effort,
``gain_kp`` and ``gain_kd`` commands to ``motor_1`` and ``motor_2`` at the
same time.

Configuration
-------------

The controllers are declared in ``config/odri_dual_motor_testbed_controllers.yaml``:

.. code-block:: yaml

   /**:
     controller_manager:
       ros__parameters:
         update_rate: 100  # Hz

         joint_state_broadcaster:
           type: joint_state_broadcaster/JointStateBroadcaster

         odri_forward_command_controller:
           type: odri_forward_command_controller/OdriForwardCommandController

     odri_forward_command_controller:
       ros__parameters:
         joints:
           - motor_1
           - motor_2
         initial_command:
           motor_1: {position: 0.0, velocity: 0.0, effort: 0.0, gain_kp: 0.0, gain_kd: 0.0}
           motor_2: {position: 0.0, velocity: 0.0, effort: 0.0, gain_kp: 0.0, gain_kd: 0.0}

``initial_command`` is applied when the controller is activated, before any
message is received. With all gains at zero, the motors do not produce any
torque at start-up.

To use another configuration file, pass ``runtime_config_package`` and
``controllers_file`` to the launch file (see :doc:`launch_files`).

Command topic
-------------

The controller listens on
``/odri_forward_command_controller/commands``
(``std_msgs/msg/Float64MultiArray``). With two joints, ``data`` must hold
exactly 10 values, grouped by interface:

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Indices
     - Content
     - Order
   * - 0–1
     - position (rad)
     - ``motor_1``, ``motor_2``
   * - 2–3
     - velocity (rad/s)
     - ``motor_1``, ``motor_2``
   * - 4–5
     - effort
     - ``motor_1``, ``motor_2``
   * - 6–7
     - ``gain_kp``
     - ``motor_1``, ``motor_2``
   * - 8–9
     - ``gain_kd``
     - ``motor_1``, ``motor_2``

A message with a different number of values is ignored. A ``NaN`` value
leaves the corresponding command unchanged.

The hardware interface limits each command. See the ``hardware_interfaces``
page of the ``odri_dual_motor_testbed_description`` documentation for the
ranges.

Examples
--------

Hold both motors at position 0 with a small PD gain:

.. code-block:: bash

   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  5.0, 5.0,  0.1, 0.1]}"

Move ``motor_1`` to 0.3 rad and leave every command of ``motor_2`` unchanged:

.. code-block:: bash

   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.3, .nan,  0.0, .nan,  0.0, .nan,  5.0, .nan,  0.1, .nan]}"

Send the zero command (no torque) before stopping:

.. code-block:: bash

   ros2 topic pub --once /odri_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
