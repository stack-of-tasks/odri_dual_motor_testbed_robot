Board state
===========

The controller claims all the state interfaces of the robot (read-only, so
they stay available to other controllers such as ``joint_state_broadcaster``)
and publishes them at each update on ``~/board_state``, i.e.
``/motkin_forward_command_controller/board_state``, with the
``motkin_dual_motor_testbed_msgs/msg/BoardState`` message:

.. code-block:: text

   std_msgs/Header header
   string[] name
   float64[] position
   float64[] velocity
   float64[] effort
   float64[] gain_kp
   float64[] gain_kd
   uint32 clock
   uint32 latest_command_index
   uint8 flags

The joint part follows ``sensor_msgs/msg/JointState``: the arrays have the
length of ``name`` and their i-th entry refers to ``name[i]``. The joints in
``joints`` come first, in that order, followed by the other joints exporting
a ``position``, ``velocity``, ``effort``, ``gain_kp`` or ``gain_kd`` state
interface (e.g. the passive joints of the five-bar), in the order the hardware
exports them. A state interface that a joint does not export is reported as
``NaN``: in Gazebo, ``gain_kp`` and ``gain_kd`` are not exported.

``clock``, ``latest_command_index`` and ``flags`` are read from the state
interfaces of the GPIO named by the ``gpio_name`` parameter, which must have
the data types ``uint32``, ``uint32`` and ``uint8``. They are 0 if the GPIO is
not exported.

.. code-block:: bash

   ros2 topic echo /motkin_forward_command_controller/board_state
