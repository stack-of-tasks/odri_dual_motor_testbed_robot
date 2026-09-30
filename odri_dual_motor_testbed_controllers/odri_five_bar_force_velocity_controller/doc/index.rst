odri_five_bar_force_velocity_controller
=======================================

A ``ros2_control`` controller that moves the five-bar of the ODRI dual motor
testbed in response to a force applied at its end point P, with the control
law

.. math::

   \dot q = J^T f_c

where J is the velocity Jacobian of P with respect to the motor angles and
f\ :sub:`c` the contact force at P.

The controller claims the command interfaces of ``motor_1`` and ``motor_2``
directly. The same law is implemented as a standalone Python node, which goes
through ``odri_forward_command_controller``, in
``odri_five_bar_force_velocity_py``. The
``odri_dual_motor_testbed_haptic_pair`` package runs this controller on two
simulated robots.

Quick start, with the testbed running:

.. code-block:: bash

   ros2 control load_controller --set-state active odri_five_bar_fv

   ros2 topic pub -r 20 /odri_five_bar_fv/contact_force geometry_msgs/msg/WrenchStamped \
     "{wrench: {force: {x: 1.0, y: 0.0, z: 0.0}}}"

The controller must be declared in the ``controller_manager`` configuration
first (see :doc:`configuration`), and ``odri_forward_command_controller``
must be inactive, since both claim the same interfaces.

.. toctree::
   :maxdepth: 2
   :caption: Contents

   control_law
   configuration
