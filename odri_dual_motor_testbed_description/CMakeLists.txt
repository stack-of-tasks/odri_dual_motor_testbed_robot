cmake_minimum_required(VERSION 3.5)
project(odri_dual_motor_testbed_description)

find_package(ament_cmake_auto REQUIRED)


ament_auto_find_build_dependencies()


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

install(DIRECTORY
  config launch  meshes robots ros2_control urdf gazebo
  DESTINATION share/${PROJECT_NAME})

ament_package()
