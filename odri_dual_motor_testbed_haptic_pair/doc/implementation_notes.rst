Implementation notes
====================

Params YAML must use the ``/**`` wildcard once namespaced
---------------------------------------------------------

A bare ``controller_manager:`` top-level key in the params YAML only matches
a node at ``/controller_manager`` (root namespace). Once the node is
namespaced to ``/leader`` or ``/follower``, that key silently stops
matching. Controllers then fail to load with
``The 'type' param was not defined for '<controller>'``, and nothing says the
namespace is the cause. The fix is to key everything under ``/**:``, as
``haptic_pair_controllers.yaml`` does.

Gazebo entity names must be unique
----------------------------------

``robot_spawn.launch.py`` names each model ``<namespace>_<robot_model>``
(passed to ``ros_gz_sim create`` with ``-name``). With two robots spawned at
runtime under the same name, only the first one comes up.
