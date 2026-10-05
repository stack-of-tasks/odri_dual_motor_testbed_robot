Launch files
============

``show.launch.py``
------------------

Displays a robot model in RViz and lets you move its joints by hand. It
starts:

* ``robot_state_publisher``, through ``robot_state_publisher.launch.py``;
* ``joint_state_publisher_gui``, with one slider per non-fixed joint;
* ``rviz2``, with ``rviz/display_motkin_dual_motor_testbed.rviz``.

.. code-block:: bash

   # Five-bar linkage (default)
   ros2 launch motkin_dual_motor_testbed_description show.launch.py

   # Dual flywheel
   ros2 launch motkin_dual_motor_testbed_description show.launch.py robot_model:=dual_flywheel

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 30 45

   * - Argument
     - Default
     - Description
   * - ``robot_model``
     - ``fivebar_2dof``
     - Model to load: ``fivebar_2dof`` or ``dual_flywheel``.
   * - ``rviz_config_file``
     - ``rviz/display_motkin_dual_motor_testbed.rviz``
     - RViz configuration file.

The screenshots in :doc:`robot_models` were taken with this launch file.

``robot_state_publisher.launch.py``
-----------------------------------

Runs xacro on the file matching ``robot_model`` and starts
``robot_state_publisher`` with the result as ``robot_description``. The
bringup and Gazebo packages include it.

.. list-table:: Arguments
   :header-rows: 1
   :widths: 25 30 45

   * - Argument
     - Default
     - Description
   * - ``robot_model``
     - ``fivebar_2dof``
     - Model to load: ``fivebar_2dof`` or ``dual_flywheel``.

The model-to-file mapping is the ``ROBOT_MODEL_XACRO_FILES`` dictionary at the
top of the launch file:

.. code-block:: python

   ROBOT_MODEL_XACRO_FILES = {
       "fivebar_2dof": "fivebar_2dof_robot.urdf.xacro",
       "dual_flywheel": "dual_flywheel_robot.urdf.xacro",
   }

To add a new robot model, put its top-level xacro in ``robots/`` and add an
entry to this dictionary. Also add it to the ``choices`` of the
``robot_model`` argument in ``show.launch.py``.
