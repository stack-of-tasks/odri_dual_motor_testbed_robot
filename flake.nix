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
