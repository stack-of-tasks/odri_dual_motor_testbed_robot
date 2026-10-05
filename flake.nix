{
  description = "ROS-2 package to handle the MOTKIN dual motor testbed robot";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";
        rosOverrideAttrs = {
          motkin-dual-motor-testbed-bringup = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_bringup;
            };
          };
          motkin-five-bar-force-velocity-controller = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_controller;
            };
          };
          motkin-five-bar-force-velocity-py = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_controllers/motkin_five_bar_force_velocity_py;
            };
          };
          motkin-forward-command-controller = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_controllers/motkin_forward_command_controller;
            };
          };
          motkin-dual-motor-testbed-description = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_description;
            };
          };
          motkin-dual-motor-testbed-gazebo = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_gazebo;
            };
          };
          motkin-dual-motor-testbed-haptic-pair = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_haptic_pair;
            };
          };
          motkin-dual-motor-testbed-hardware = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_hardware;
            };
          };
          motkin-dual-motor-testbed-robot = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./motkin_dual_motor_testbed_robot;
            };
          };
        };
      }
    );
}
