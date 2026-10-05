motkin_dual_motor_testbed_hardware
==================================

``ros2_control`` hardware plugins for the MOTKIN dual motor testbed.

On the real robot only ``motor_1`` and ``motor_2`` have encoders. The
``FiveBarSystem`` plugin wraps the plugin that drives the motor board and
adds the two passive joints of the five-bar, ``passive_1`` and
``passive_2``, as state-only interfaces. Their positions and velocities are
computed from the motor encoders with the exact direct geometric model of the
five-bar. ``joint_state_broadcaster`` therefore publishes the whole five-bar,
as it does in simulation.

``FiveBarSystem`` is selected by the ``five_bar="true"`` argument of the
``motkin_ros2_control`` macro in
``motkin_dual_motor_testbed_description``. The ``fivebar_2dof`` model sets it;
the ``dual_flywheel`` model does not, and uses the board plugin directly.

.. toctree::
   :maxdepth: 2
   :caption: Contents

   package_structure
   five_bar_system
   passive_joint_model
