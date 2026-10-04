{
  description = "ROS-2 package to handle the ODRI dual motor testbed robot";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";
        rosOverrideAttrs = {
          odri-dual-motor-testbed-bringup = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_bringup;
            };
          };
          odri-five-bar-force-velocity-controller = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_controllers/odri_five_bar_force_velocity_controller;
            };
          };
          odri-five-bar-force-velocity-py = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_controllers/odri_five_bar_force_velocity_py;
            };
          };
          odri-forward-command-controller = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_controllers/odri_forward_command_controller;
            };
          };
          odri-dual-motor-testbed-description = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_description;
            };
          };
          odri-dual-motor-testbed-gazebo = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_gazebo;
            };
          };
          odri-dual-motor-testbed-haptic-pair = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_haptic_pair;
            };
          };
          odri-dual-motor-testbed-hardware = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_hardware;
            };
          };
          odri-dual-motor-testbed-robot = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./odri_dual_motor_testbed_robot;
            };
          };
        };
      }
    );
}
