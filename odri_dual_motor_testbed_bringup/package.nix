{
  lib,
  buildRosPackage,

  ament-cmake,

  ament-cmake-auto,
  ament-cmake-cppcheck,
  ament-cmake-cpplint,
  ament-cmake-copyright,
  ament-cmake-lint-cmake,
  ament-cmake-pep257,
  ament-cmake-uncrustify,
  ament-cmake-xmllint,
  ament-cpplint,
  ament-cppcheck,
  ament-copyright,
  ament-lint-auto,
  ament-lint-cmake,
  ament-pep257,
  ament-uncrustify,
  ament-lint-common,
  ament-xmllint,
}:
buildRosPackage {
  pname = "odri-dual-motor-testbed-bringup";
  version = "1.0.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./config
      ./package.xml
      ./launch
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

  propagatedBuildInputs = [
    # keep-sorted start
    # keep-sorted end
  ];
  checkInputs = [
    # keep-sorted start
    ament-cmake-copyright
    ament-cmake-cppcheck
    ament-cmake-cpplint
    ament-cmake-lint-cmake
    ament-cmake-pep257
    ament-cmake-uncrustify
    ament-cmake-xmllint
    ament-lint-auto
    ament-lint-common
    # keep-sorted end
  ];
  nativeCheckInputs = [
    # keep-sorted start
    ament-copyright
    ament-cppcheck
    ament-cpplint
    ament-lint-cmake
    ament-pep257
    ament-uncrustify
    ament-xmllint
    # keep-sorted end
  ];

  doCheck = false; # TODO: cppcheck + cpplint + uncrustify

  meta = {
    description = "{{ pkg.description }}";
    license = with lib.licenses; [ asl20 ];
    homepage = "https://github.com/stack-of-tasks/odri_gz_ros2_control";
    platforms = lib.platforms.linux;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
