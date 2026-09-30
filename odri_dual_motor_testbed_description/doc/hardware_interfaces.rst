ros2_control hardware interfaces
================================

The ``gz_sim`` xacro argument of the top-level robot files selects the
``ros2_control`` system that is included (see :doc:`robot_models`).

Real hardware: Pico dual DRV8316C board
---------------------------------------

With ``gz_sim:=false`` (the default), the robot files include
``ros2_control/system_pico_dual_drv8316c.ros2_control.xacro`` and call the
``pico_dual_drv8316c_ros2_control`` macro. The plugin
``pico_dual_drv8316c_hardware_interface/SystemPicoDualDrv8316CHardware``
talks to a Pico dual PMSM / DRV8316C board over USB serial.

With ``five_bar="true"`` (set by the ``fivebar_2dof`` model), the macro
loads ``odri_dual_motor_testbed_hardware/FiveBarSystem`` instead. It wraps
the Pico plugin, passes it the motor joints, and adds ``passive_1`` and
``passive_2`` as state-only joints (``position``, ``velocity``, ``effort``),
computed from the motor encoders with the direct geometric model of the
five-bar. The geometry parameters written by the macro must match those of
the ``FiveBarClosurePlugin`` in ``gazebo/gazebo_fivebar_2dof.urdf.xacro``.
See the ``odri_dual_motor_testbed_hardware`` documentation.

.. list-table:: Macro parameters
   :header-rows: 1
   :widths: 25 20 55

   * - Parameter
     - Default
     - Description
   * - ``name``
     - (required)
     - Name of the ``ros2_control`` system.
   * - ``left_joint_name``
     - ``motor_1``
     - Joint driven by the first motor.
   * - ``right_joint_name``
     - ``motor_2``
     - Joint driven by the second motor.
   * - ``serial_port``
     - ``''``
     - Serial device. If empty, the driver searches ``/dev/serial/by-id``,
       then ``/dev/ttyACM*``, then ``/dev/ttyUSB*``.
   * - ``baud_rate``
     - ``115200``
     - Serial baud rate.
   * - ``timeout_ms``
     - ``20``
     - Serial read timeout, in milliseconds.
   * - ``five_bar``
     - ``false``
     - ``true`` wraps the board plugin in ``FiveBarSystem`` and declares the
       passive joints ``passive_1`` and ``passive_2``. Only for
       ``fivebar_2dof``.

Each motor joint exposes:

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Interface
     - Command range
     - State
   * - ``position``
     - [−0.9, 0.9]
     - yes
   * - ``velocity``
     - [−1, 1]
     - yes
   * - ``effort``
     - [−1, 1]
     - yes
   * - ``gain_kp``
     - [−1000, 1000]
     - yes
   * - ``gain_kd``
     - [−1000, 1000]
     - yes

Simulation: Gazebo Harmonic
---------------------------

With ``gz_sim:=true``, each robot file includes its own Gazebo file,
``gazebo/gazebo_fivebar_2dof.urdf.xacro`` or
``gazebo/gazebo_dual_flywheel.urdf.xacro``, and calls the
``odri_dual_motor_testbed_gazebo`` macro it defines. Both declare:

* a ``GazeboOdriSimSystem`` ``ros2_control`` system
  (``odri_gz_ros2_control/GazeboOdriSimSystem``). ``motor_1`` and ``motor_2``
  have the ``position``, ``velocity``, ``effort``, ``gain_kp`` and ``gain_kd``
  command interfaces;
* the ``odri_gz_ros2_control`` Gazebo system plugin, which loads
  ``$(arg controller_params_file)`` in the ``$(arg robot_namespace)``
  namespace.

The five-bar file also declares ``passive_1`` and ``passive_2`` as
state-only joints, and the ``FiveBarClosurePlugin``, which closes the
five-bar loop (see :doc:`robot_models`).

