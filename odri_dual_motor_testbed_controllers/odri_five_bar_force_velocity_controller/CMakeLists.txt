cmake_minimum_required(VERSION 3.16)
project(odri_five_bar_force_velocity_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    geometry_msgs
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# --- Parameter library ---
generate_parameter_library(
  odri_five_bar_force_velocity_controller_parameters
  src/odri_five_bar_force_velocity_controller_parameters.yaml)

# --- Controller library ---
add_library(${PROJECT_NAME} SHARED
            src/odri_five_bar_force_velocity_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/odri_five_bar_force_velocity_controller>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC odri_five_bar_force_velocity_controller_parameters
         controller_interface::controller_interface
         hardware_interface::hardware_interface
         pluginlib::pluginlib
         rclcpp::rclcpp
         rclcpp_lifecycle::rclcpp_lifecycle
         realtime_tools::realtime_tools
         ${geometry_msgs_TARGETS})

pluginlib_export_plugin_description_file(
  controller_interface odri_five_bar_force_velocity_controller_plugin.xml)

# --- Install ---
install(DIRECTORY include/
        DESTINATION include/odri_five_bar_force_velocity_controller)
install(
  TARGETS ${PROJECT_NAME} odri_five_bar_force_velocity_controller_parameters
  EXPORT export_odri_five_bar_force_velocity_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_five_bar_kinematics test/test_five_bar_kinematics.cpp)
  target_include_directories(
    test_five_bar_kinematics
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
endif()

ament_export_targets(export_odri_five_bar_force_velocity_controller
                     HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
