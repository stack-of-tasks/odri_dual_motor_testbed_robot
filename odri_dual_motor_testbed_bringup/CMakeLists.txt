cmake_minimum_required(VERSION 3.22)
project(odri_dual_motor_testbed_bringup)

find_package(ament_cmake_auto REQUIRED)
find_package(odri_dual_motor_testbed_description)

ament_auto_find_build_dependencies()

# ##############################################################################
# Testing ##
# ##############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_lint_cmake)
  ament_lint_auto_find_test_dependencies()

endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

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
