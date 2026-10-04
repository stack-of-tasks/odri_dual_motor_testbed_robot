cmake_minimum_required(VERSION 3.16)
project(odri_dual_motor_testbed_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib rclcpp
                                 rclcpp_lifecycle)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/five_bar_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface
                                         odri_dual_motor_testbed_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_five_bar_passive_joints
                  test/test_five_bar_passive_joints.cpp)
  target_include_directories(test_five_bar_passive_joints
                             PRIVATE ${PROJECT_SOURCE_DIR}/include)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Documentation ##
# ##############################################################################

find_program(ROSDOC2 rosdoc2 REQUIRED)
set(DOC_WORK_DIR ${PROJECT_BINARY_DIR}/rosdoc2)
set(DOC_INDEX ${DOC_WORK_DIR}/output/${PROJECT_NAME}/index.html)
file(GLOB_RECURSE DOC_SOURCES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/doc/*
     ${PROJECT_SOURCE_DIR}/include/* ${PROJECT_SOURCE_DIR}/README.md)
add_custom_command(
  OUTPUT ${DOC_INDEX}
  COMMAND
    ${ROSDOC2} build --package-path ${PROJECT_SOURCE_DIR} --output-directory
    ${DOC_WORK_DIR}/output --doc-build-directory ${DOC_WORK_DIR}/build
    --cross-reference-directory ${DOC_WORK_DIR}/cross_reference
  DEPENDS ${DOC_SOURCES} ${PROJECT_SOURCE_DIR}/package.xml
  COMMENT "Building the ${PROJECT_NAME} documentation with rosdoc2"
  VERBATIM)
add_custom_target(${PROJECT_NAME}_doc ALL DEPENDS ${DOC_INDEX})
install(DIRECTORY ${DOC_WORK_DIR}/output/${PROJECT_NAME}/
        DESTINATION share/${PROJECT_NAME}/doc)

ament_package()
