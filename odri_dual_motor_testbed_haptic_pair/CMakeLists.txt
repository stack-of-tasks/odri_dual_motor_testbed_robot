cmake_minimum_required(VERSION 3.22)
project(odri_dual_motor_testbed_haptic_pair)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module odri_dual_motor_testbed_haptic_pair (contact_force_relay).
ament_python_install_package(${PROJECT_NAME})

# Executable for ros2 run / launch files.
install(
  PROGRAMS ${PROJECT_NAME}/contact_force_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME contact_force_relay)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)

# ##############################################################################
# Documentation ##
# ##############################################################################

# Built with rosdoc2 and installed in share/${PROJECT_NAME}/doc. Enable with
# colcon build --cmake-args -DBUILD_DOCS=ON, or BUILD_DOCS=ON colcon build.
option(BUILD_DOCS "Build the documentation with rosdoc2" OFF)
if(BUILD_DOCS OR "$ENV{BUILD_DOCS}")
  find_program(ROSDOC2 rosdoc2 REQUIRED)
  set(DOC_WORK_DIR ${PROJECT_BINARY_DIR}/rosdoc2)
  set(DOC_INDEX ${DOC_WORK_DIR}/output/${PROJECT_NAME}/index.html)
  file(GLOB_RECURSE DOC_SOURCES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/doc/*
       ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/*.py
       ${PROJECT_SOURCE_DIR}/README.md)
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
endif()

ament_package()
