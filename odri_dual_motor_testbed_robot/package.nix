{
  lib,
  buildRosPackage,

  ament-cmake,
}:
buildRosPackage {
  pname = "odri-dual-motor-testbed-robot";
  version = "1.0.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./package.xml
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
