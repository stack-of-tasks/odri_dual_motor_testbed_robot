{
  lib,
  buildRosPackage,

  ament-cmake,
  ament-cmake-auto,

  ament-cmake-cppcheck,
  ament-cmake-cpplint,
  ament-cmake-copyright,
  ament-cmake-pep257,
  ament-cmake-lint-cmake,
  ament-cmake-xmllint,
  ament-cmake-uncrustify,
  ament-cpplint,
  ament-cppcheck,
  ament-copyright,
  ament-lint-auto,
  ament-pep257,
  ament-uncrustify,
  ament-lint-cmake,
  ament-lint-common,
  ament-xmllint,
}:
buildRosPackage {
  pname = "odri-dual-motor-testbed-description";
  version = "1.0.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./config
      ./gazebo
      ./launch
      ./meshes
      ./odri_dual_motor_testbed.rviz
      ./package.xml
      ./robots
      ./ros2_control
      ./urdf
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
    ament-cmake-auto
  ];

  propagatedBuildInputs = [ ];
  checkInputs = [
    # keep-sorted start
    ament-cmake-copyright
    ament-cmake-cppcheck
    ament-cmake-cpplint
    ament-cmake-pep257
    ament-cmake-lint-cmake
    ament-cmake-xmllint
    ament-cmake-uncrustify
    ament-lint-auto
    ament-lint-common
    # keep-sorted end
  ];
  nativeCheckInputs = [
    # keep-sorted start
    ament-pep257
    ament-uncrustify
    ament-copyright
    ament-cppcheck
    ament-cpplint
    ament-lint-cmake
    ament-xmllint
    # keep-sorted end
  ];

  doCheck = false; # TODO: cppcheck + cpplint + uncrustify

  meta = {
    description = "Package describing the ODRI dual motor testbed robot";
    license = with lib.licenses; [ asl20 ];
    homepage = "https://github.com/stack-of-tasks/odri_dual_motor_testbed_robot";
    platforms = lib.platforms.linux;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
