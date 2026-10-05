cmake_minimum_required(VERSION 3.16)
project(motkin_forward_command_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

# --- Parameter library ---
generate_parameter_library(motkin_forward_command_controller_parameters
                           src/motkin_forward_command_controller_parameters.yaml
)

# --- Controller library ---
add_library(${PROJECT_NAME} SHARED src/motkin_forward_command_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/motkin_forward_command_controller>
)
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        motkin_forward_command_controller_parameters
        controller_interface::controller_interface
        hardware_interface::hardware_interface
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        realtime_tools::realtime_tools
        ${std_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(
  controller_interface motkin_forward_command_controller_plugin.xml
)

# --- Install ---
install(
    DIRECTORY include/
    DESTINATION include/motkin_forward_command_controller
)
install(
    TARGETS ${PROJECT_NAME} motkin_forward_command_controller_parameters
    EXPORT export_motkin_forward_command_controller
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_gmock REQUIRED)

    ament_add_gmock(test_motkin_forward_command_controller
                    test/test_motkin_forward_command_controller.cpp
    )
    target_include_directories(
        test_motkin_forward_command_controller
        PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    )
    target_link_libraries(
        test_motkin_forward_command_controller
        ${PROJECT_NAME}
    )
endif()

ament_export_targets(export_motkin_forward_command_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Documentation ##
# ##############################################################################

set(BUILD_DOCS_DEFAULT OFF)
find_program(ROSDOC2 rosdoc2 QUIET)
if(ROSDOC2)
    set(BUILD_DOCS_DEFAULT ON)
endif()
option(BUILD_DOCS "Build the documentation with rosdoc2" ${BUILD_DOCS_DEFAULT})
if(BUILD_DOCS OR "$ENV{BUILD_DOCS}")
    set(DOC_WORK_DIR ${PROJECT_BINARY_DIR}/rosdoc2)
    set(DOC_INDEX ${DOC_WORK_DIR}/output/${PROJECT_NAME}/index.html)
    file(
        GLOB_RECURSE DOC_SOURCES
        CONFIGURE_DEPENDS
        ${PROJECT_SOURCE_DIR}/doc/*
        ${PROJECT_SOURCE_DIR}/include/*
        ${PROJECT_SOURCE_DIR}/README.md
    )
    add_custom_command(
        OUTPUT ${DOC_INDEX}
        COMMAND
            ${ROSDOC2} build --package-path ${PROJECT_SOURCE_DIR}
            --output-directory ${DOC_WORK_DIR}/output --doc-build-directory
            ${DOC_WORK_DIR}/build --cross-reference-directory
            ${DOC_WORK_DIR}/cross_reference
        DEPENDS ${DOC_SOURCES} ${PROJECT_SOURCE_DIR}/package.xml
        COMMENT "Building the ${PROJECT_NAME} documentation with rosdoc2"
        VERBATIM
    )
    add_custom_target(${PROJECT_NAME}_doc ALL DEPENDS ${DOC_INDEX})
    install(
        DIRECTORY ${DOC_WORK_DIR}/output/${PROJECT_NAME}/
        DESTINATION share/${PROJECT_NAME}/doc
    )
endif()

ament_package()
