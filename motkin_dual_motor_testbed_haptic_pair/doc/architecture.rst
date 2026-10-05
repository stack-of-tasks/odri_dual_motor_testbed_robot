Architecture
============

The launch file starts one world with ``motkin_dual_motor_testbed_gazebo``'s
``gz_world.launch.py``, then includes ``motkin_dual_motor_testbed_gazebo``'s
``robot_spawn.launch.py`` twice, with ``namespace:=leader`` and
``namespace:=follower``. Each robot gets its own ``controller_manager``
(``/leader/controller_manager``, ``/follower/controller_manager``), which
loads the *existing* ``motkin_five_bar_force_velocity_controller`` (from
``motkin_five_bar_force_velocity_controller``). That controller already
implements the admittance law q̇ = J\ :sup:`T` f\ :sub:`c`, which turns a
sensed contact force into motion for this mechanism.

The only new code is ``contact_force_relay``: it republishes whatever
``geometry_msgs/WrenchStamped`` the leader receives on
``/leader/motkin_five_bar_force_velocity_controller/contact_force`` onto
``/follower/motkin_five_bar_force_velocity_controller/contact_force``. Since the
follower runs the identical controller, it reproduces the same force -- and,
by construction of the shared admittance law, the same motion -- with no new
control algorithm.

.. code-block:: text

   [external force] -> leader/contact_force -> leader controller -> leader motion
                               |
                        contact_force_relay
                               v
                        follower/contact_force -> follower controller -> follower motion

``contact_force_relay`` node
----------------------------

.. list-table:: Parameters
   :header-rows: 1
   :widths: 25 45 30

   * - Parameter
     - Default
     - Description
   * - ``input_topic``
     - ``/leader/motkin_five_bar_force_velocity_controller/contact_force``
     - Topic the force is read from.
   * - ``output_topic``
     - ``/follower/motkin_five_bar_force_velocity_controller/contact_force``
     - Topic the force is republished on.
