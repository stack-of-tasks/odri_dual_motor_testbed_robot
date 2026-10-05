Architecture
============

The same robot description, controllers and commands are used on the real
robot and in simulation. Only the ``ros2_control`` hardware system changes,
selected by the ``gz_sim`` xacro argument of the robot files.

.. code-block:: text

                         robots/<robot_model>_robot.urdf.xacro
                                        │
                  gz_sim:=false         │          gz_sim:=true
          ┌─────────────────────────────┴──────────────────────────────┐
          │                                                            │
   Real robot (bringup)                                    Gazebo (gazebo)
   ros2_control_node                                       motkin_gz_ros2_control
     FiveBarSystem (five-bar only)                           GazeboMotkinSimSystem
       └─ SystemPicoDualDrv8316CHardware ── USB ── board     FiveBarClosurePlugin
          │                                                            │
          └──────────────────── controller_manager (100 Hz) ───────────┘
                     │                                   │
          joint_state_broadcaster          motkin_forward_command_controller
                     │                     or motkin_five_bar_force_velocity_controller
               /joint_states                             │
                     │                 /motkin_forward_command_controller/commands
            robot_state_publisher

Joints
------

Both models have two actuated joints, ``motor_1`` and ``motor_2``. Each
exposes the ``position``, ``velocity``, ``effort``, ``gain_kp`` and
``gain_kd`` command interfaces, which the motor board (or its simulation)
turns into a torque:

.. math::

   \tau = \tau_{cmd} + K_p (q_{cmd} - q) + K_d (\dot q_{cmd} - \dot q)

The five-bar also has two passive joints, ``passive_1`` and ``passive_2``,
with state interfaces only:

* on the real robot they are computed from the motor encoders by
  ``FiveBarSystem`` (``motkin_dual_motor_testbed_hardware``);
* in Gazebo they come from the physics simulation, the loop being closed by
  ``FiveBarClosurePlugin`` (``motkin_dual_motor_testbed_gazebo``).

In both cases ``joint_state_broadcaster`` publishes the four joints.

Five-bar geometric model
------------------------

``five_bar_mgd_spec.md``, at the root of the repository, derives the exact
direct geometric model of the five-bar: the position of the end point and of
the passive joints as a function of the motor angles. It is implemented in
four places, whose parameters must stay identical:

* ``FiveBarClosurePlugin``, to assemble the loop at start-up
  (``motkin_dual_motor_testbed_description/gazebo/gazebo_fivebar_2dof.urdf.xacro``);
* ``FiveBarSystem``
  (``motkin_dual_motor_testbed_description/ros2_control/system_pico_dual_drv8316c.ros2_control.xacro``);
* ``motkin_five_bar_force_velocity_controller`` and
  ``motkin_five_bar_force_velocity_py`` (their ``geometry.*`` parameters).
