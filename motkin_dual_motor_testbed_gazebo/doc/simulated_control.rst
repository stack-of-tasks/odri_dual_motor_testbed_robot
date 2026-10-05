Controlling the simulated robot
===============================

The simulation exposes the same ``ros2_control`` interfaces as the real
testbed, so the same controllers and commands work in both.

Controllers
-----------

``config/forward_command_controller.yaml`` is loaded by the
``motkin_gz_ros2_control`` Gazebo plugin (through the ``controller_params_file``
xacro argument). It declares, at 100 Hz:

* ``joint_state_broadcaster``, which publishes ``/joint_states`` for
  ``motor_1``, ``motor_2`` and, on the five-bar, ``passive_1`` and
  ``passive_2``;
* ``motkin_forward_command_controller`` on ``motor_1`` and ``motor_2``, with an
  ``initial_command`` where every gain is 0, so the motors produce no torque
  until a command is received.

Commands are sent exactly as on the real robot, on
``/motkin_forward_command_controller/commands``. See the documentation of
``motkin_forward_command_controller`` for the message layout. For example:

.. code-block:: bash

   ros2 topic pub --once /motkin_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.3, -0.3,  0.0, 0.0,  0.0, 0.0,  2.0, 2.0,  0.02, 0.02]}"

Motor model
-----------

The ``GazeboMotkinSimSystem`` hardware interface applies the torque law of the
MOTKIN motor boards on each actuated joint:

.. math::

   \tau = \tau_{cmd} + K_p (q_{cmd} - q) + K_d (\dot q_{cmd} - \dot q)

where τ\ :sub:`cmd`, q\ :sub:`cmd`, q̇\ :sub:`cmd`, K\ :sub:`p` and
K\ :sub:`d` are the ``effort``, ``position``, ``velocity``, ``gain_kp`` and
``gain_kd`` commands. As on the real board, this PD loop runs at every
physics step (1 kHz) on the current joint state, independently of the 100 Hz
controller update rate.

The motors of the testbed have a very small inertia, I ≈ 1.4·10\ :sup:`-5`
kg·m² for the rotor alone. Applied explicitly over a 1 ms step, the damping
term would be unstable as soon as K\ :sub:`d` Δt / I > 2, that is for
K\ :sub:`d` above about 0.015: the velocity would oscillate between the joint
limits instead of following the command. The damping term is therefore
applied implicitly:

.. math::

   \tau = \tau_{cmd} + K_p (q_{cmd} - q)
        + \frac{K_d}{1 + K_d \Delta t / I} (\dot q_{cmd} - \dot q)

where I is the inertia of the joint's child link about the joint axis, read
from the model at start-up and logged as ``damping inertia``. It is a lower
bound of the inertia seen by the motor, so the scheme is stable for any
K\ :sub:`d`. For K\ :sub:`d` Δt ≪ I the law is unchanged. The effort and
K\ :sub:`p` terms are never modified, so static equilibria are exact.

On the five-bar, the passive joints have no command interface. Their state
comes from the physics simulation, with the loop closed by the
:doc:`five_bar_closure_plugin`.

Checking that it runs
---------------------

.. code-block:: bash

   # Both controllers should be "active"
   ros2 control list_controllers

   # Joint states, stamped with the simulation time
   ros2 topic echo /joint_states
