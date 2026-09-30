odri_dual_motor_testbed_bringup
===============================

Launch files and controller configuration to start the ODRI dual motor
testbed on real hardware with ``ros2_control``.

The robot description, including the ``ros2_control`` hardware interface, is
provided by ``odri_dual_motor_testbed_description``. This package starts the
``controller_manager`` with that description, spawns the controllers and opens
RViz.

Quick start:

.. code-block:: bash

   ros2 launch odri_dual_motor_testbed_bringup odri_dual_motor_testbed_ctrl.launch.py robot_model:=fivebar_2dof

.. toctree::
   :maxdepth: 2
   :caption: Contents

   installation
   starting_the_testbed
   sending_commands
   launch_files
