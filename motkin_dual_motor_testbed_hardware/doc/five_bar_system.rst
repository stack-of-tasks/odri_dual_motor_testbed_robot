FiveBarSystem
=============

``motkin_dual_motor_testbed_hardware/FiveBarSystem`` is a
``hardware_interface::SystemInterface`` that wraps another one, the *inner
plugin*.

How it works
------------

At initialisation, the plugin splits the joints declared under
``<ros2_control>``:

* the two passive joints stay in ``FiveBarSystem``. They must have no command
  interface, and only the ``position``, ``velocity`` and ``effort`` state
  interfaces;
* every other joint is passed to the inner plugin, which is loaded with
  pluginlib under the name ``<system name>_inner``.

Then:

* every lifecycle transition, ``write()`` and command mode switch is forwarded
  to the inner plugin;
* the state and command interfaces of the inner plugin are exported as they
  are, and the passive joint state interfaces are added to them;
* after each ``read()`` of the inner plugin, the passive joint positions and
  velocities are recomputed from the ``motor_1`` / ``motor_2`` position and
  velocity states (see :doc:`passive_joint_model`). The passive efforts are
  always 0.

Both interface export APIs of ``ros2_control`` are supported, so the inner
plugin may use either the legacy one (like the Pico board plugin) or the
framework-managed one (like ``mock_components/GenericSystem``).

Passive positions are unwrapped against the previous value, so they stay
continuous when the ``atan2`` in the model crosses ±π.

Error handling
--------------

* **Outside the workspace** (no assembly of the loop exists for the measured
  motor angles): an error is logged once, the passive positions keep their
  last value and their velocities are set to 0. A message is logged when the
  robot comes back into the workspace.
* **Singular configuration** (the velocity map cannot be inverted): the
  passive velocities are set to 0 and a warning is logged, at most once per
  second.
* The plugin fails at start-up if a passive joint is missing, has a command
  interface, or if the inner plugin does not export
  ``<motor>/position``.

Hardware parameters
-------------------

All parameters are optional.

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Parameter
     - Default
     - Description
   * - ``inner_plugin``
     - ``pico_dual_drv8316c_hardware_interface/SystemPicoDualDrv8316CHardware``
     - Plugin that drives the motors.
   * - ``motor_joint1``, ``motor_joint2``
     - ``motor_1``, ``motor_2``
     - Actuated joints (θ\ :sub:`1`, θ\ :sub:`2`).
   * - ``passive_joint1``, ``passive_joint2``
     - ``passive_1``, ``passive_2``
     - Passive joints (β\ :sub:`1`, β\ :sub:`2`).
   * - ``a_x``, ``a_y``
     - ``-0.0421006``, ``0.0302628``
     - Axis of ``motor_1`` in the (x, y) plane of ``case``, in meters.
   * - ``b_x``, ``b_y``
     - ``0.0578994``, ``0.0302628``
     - Axis of ``motor_2`` in the (x, y) plane of ``case``, in meters.
   * - ``l1``
     - ``0.06``
     - Crank length (motor to elbow), in meters.
   * - ``l2``
     - ``0.1``
     - Coupler length (elbow to closure point), in meters.
   * - ``phi1``, ``phi2``
     - ``1.5707963267948966``
     - Angular offsets of the cranks, in radians.
   * - ``psi1``
     - ``0.21205399999927563``
     - Angular offset of the left coupler, in radians.
   * - ``psi2``
     - ``2.511306362956145``
     - Angular offset of the right coupler, in radians.

The other hardware parameters (``serial_port``, ``baud_rate``,
``timeout_ms``) are passed through to the inner plugin.

Example
-------

This is what the ``pico_dual_drv8316c_ros2_control`` macro generates with
``five_bar="true"`` (geometry and interface limits shortened):

.. code-block:: xml

   <ros2_control name="picodualdrv8316c" type="system">
     <hardware>
       <plugin>motkin_dual_motor_testbed_hardware/FiveBarSystem</plugin>
       <param name="inner_plugin">pico_dual_drv8316c_hardware_interface/SystemPicoDualDrv8316CHardware</param>
       <param name="motor_joint1">motor_1</param>
       <param name="motor_joint2">motor_2</param>
       <param name="passive_joint1">passive_1</param>
       <param name="passive_joint2">passive_2</param>
       <param name="l1">0.06</param>
       <param name="l2">0.1</param>
       <!-- a_x a_y b_x b_y phi1 phi2 psi1 psi2 ... -->
       <param name="serial_port"></param>
     </hardware>
     <joint name="motor_1"> <!-- command and state interfaces --> </joint>
     <joint name="motor_2"> <!-- command and state interfaces --> </joint>
     <joint name="passive_1">
       <state_interface name="position"/>
       <state_interface name="velocity"/>
       <state_interface name="effort"/>
     </joint>
     <joint name="passive_2">
       <state_interface name="position"/>
       <state_interface name="velocity"/>
       <state_interface name="effort"/>
     </joint>
   </ros2_control>

.. warning::

   The geometric parameters must be the same as those of the
   ``FiveBarClosurePlugin`` in
   ``motkin_dual_motor_testbed_description/gazebo/gazebo_fivebar_2dof.urdf.xacro``,
   so that simulation and hardware report the same passive joint angles.

Testing without the board
-------------------------

Any ``SystemInterface`` can be wrapped. To check the passive joints without
the motor board, set ``inner_plugin`` to ``mock_components/GenericSystem``:
the motors then follow their position commands and the passive joints follow
the model.
