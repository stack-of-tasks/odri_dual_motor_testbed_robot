{
  description = "ROS-2 package to handle the ODRI dual motor testbed robot";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";
        rosPackages = {
          odri-dual-motor-testbed-bringup = ./odri_dual_motor_testbed_bringup/package.nix;
          odri-dual-motor-testbed-description = ./odri_dual_motor_testbed_description/package.nix;
          odri-dual-motor-testbed-gazebo = ./odri_dual_motor_testbed_gazebo/package.nix;
          odri-dual-motor-testbed-robot = ./odri_dual_motor_testbed_robot/package.nix;
        };
      }
    );
}
