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

With ``gz_sim:=true``, the robot files include ``gazebo/gazebo.urdf.xacro``
and call the ``odri_dual_motor_testbed_gazebo`` macro. It declares:

* a ``GazeboOdriSimSystem`` ``ros2_control`` system
  (``odri_gz_ros2_control/GazeboOdriSimSystem``). ``motor_1`` and ``motor_2``
  have the ``position``, ``velocity``, ``effort``, ``gain_kp`` and ``gain_kd``
  command interfaces. ``passive_1`` and ``passive_2`` are state-only;
* the ``odri_gz_ros2_control`` Gazebo system plugin, which loads
  ``$(arg controller_params_file)`` in the ``$(arg robot_namespace)``
  namespace;
* the ``FiveBarClosurePlugin``, which closes the five-bar loop (see
  :doc:`robot_models`).

