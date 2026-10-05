Configuration
=============

Parameters
----------

The parameters are declared with ``generate_parameter_library`` in
``src/motkin_forward_command_controller_parameters.yaml``.

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``joints``
     - ``[]``
     - Names of the joints to control. Read-only.
   * - ``initial_command.<joint>.position``
     - ``0.0``
     - Position command applied on activation.
   * - ``initial_command.<joint>.velocity``
     - ``0.0``
     - Velocity command applied on activation.
   * - ``initial_command.<joint>.effort``
     - ``0.0``
     - Effort command applied on activation.
   * - ``initial_command.<joint>.gain_kp``
     - ``0.0``
     - ``gain_kp`` applied on activation.
   * - ``initial_command.<joint>.gain_kd``
     - ``0.0``
     - ``gain_kd`` applied on activation.

``initial_command`` is applied when the controller is activated, before any
message is received on ``~/commands``, so that each joint starts in a known
state. With the defaults, every gain and effort is 0 and the motors produce
no torque.

Example
-------

The configuration used on the testbed
(``motkin_dual_motor_testbed_bringup/config/motkin_dual_motor_testbed_controllers.yaml``
and ``motkin_dual_motor_testbed_gazebo/config/forward_command_controller.yaml``):

.. code-block:: yaml

   /**:
     controller_manager:
       ros__parameters:
         update_rate: 100  # Hz

         joint_state_broadcaster:
           type: joint_state_broadcaster/JointStateBroadcaster

         motkin_forward_command_controller:
           type: motkin_forward_command_controller/MotkinForwardCommandController

     motkin_forward_command_controller:
       ros__parameters:
         joints:
           - motor_1
           - motor_2
         initial_command:
           motor_1: {position: 0.0, velocity: 0.0, effort: 0.0, gain_kp: 0.0, gain_kd: 0.0}
           motor_2: {position: 0.0, velocity: 0.0, effort: 0.0, gain_kp: 0.0, gain_kd: 0.0}

Building and testing
--------------------

.. code-block:: bash

   colcon build --packages-select motkin_forward_command_controller
   colcon test --packages-select motkin_forward_command_controller
   colcon test-result --verbose

The tests in ``test/test_motkin_forward_command_controller.cpp`` cover
configuration and activation, command forwarding, ``NaN`` skipping,
rejection of messages of the wrong size and the release of the interfaces on
deactivation.

Building this documentation
---------------------------

``colcon build`` builds this documentation with rosdoc2 when available,
and installs it in ``share/motkin_forward_command_controller/doc``:

.. code-block:: bash

   colcon build --packages-select motkin_forward_command_controller
   xdg-open install/motkin_forward_command_controller/share/motkin_forward_command_controller/doc/index.html

To build it by hand instead, run from the directory that contains this
package:

.. code-block:: bash

   rosdoc2 build --package-path motkin_forward_command_controller
   rosdoc2 open docs_output/motkin_forward_command_controller/index.html
