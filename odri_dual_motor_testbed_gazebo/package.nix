{
  lib,
  buildRosPackage,

  ament-cmake,

  gz-sim,
  gz-plugin,

  ament-cmake-auto,
}:
buildRosPackage {
  pname = "odri-dual-motor-testbed-gazebo";
  version = "1.0.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./config
      ./launch
      ./package.xml
      ./src
    ];
  };

  buildType = "ament_cmake";
  __structuredAttrs = true;
  strictDeps = true;

  nativeBuildInputs = [
    ament-cmake
  ];
  buildInputs = [
    ament-cmake
    gz-sim
    gz-plugin
  ];

  checkInputs = [
    ament-cmake-auto
  ];

  doCheck = true;

  meta = {
    description = "{{ pkg.description }}";
    license = with lib.licenses; [ asl20 ];
    homepage = "https://github.com/stack-of-tasks/odri_gz_ros2_control";
    platforms = lib.platforms.linux;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
