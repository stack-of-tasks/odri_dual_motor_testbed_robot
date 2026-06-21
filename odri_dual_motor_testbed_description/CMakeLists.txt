cmake_minimum_required(VERSION 3.22)
project(odri_dual_motor_testbed_description)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# ##############################################################################
# Testing ##
# ##############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
  ament_lint_auto_find_test_dependencies()

endif()

install(
  DIRECTORY launch
            meshes
            robots
            ros2_control
            urdf
            gazebo
            config
  DESTINATION share/${PROJECT_NAME})

ament_package()
