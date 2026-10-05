motkin_dual_motor_testbed_robot
=============================

Meta-package and entry point of the ``ros2_control`` software of the MOTKIN
dual motor testbed: a base with two MOTKIN motors, driven by a Raspberry Pi Pico
dual DRV8316C board, on which either a planar five-bar linkage
(``fivebar_2dof``) or two flywheels (``dual_flywheel``) are mounted.

Installing this package installs ``motkin_dual_motor_testbed_description`` and
``motkin_dual_motor_testbed_bringup``. Each package of the repository has its
own documentation; this page explains how they fit together.

.. toctree::
   :maxdepth: 2
   :caption: Contents

   packages
   architecture
   getting_started
