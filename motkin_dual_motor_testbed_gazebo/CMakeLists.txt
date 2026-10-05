cmake_minimum_required(VERSION 3.22)
project(motkin_dual_motor_testbed_gazebo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# gz-sim8 (Gazebo Harmonic) – vendored under the Jazzy distribution
find_package(
    gz-sim8
    REQUIRED
    HINTS /opt/ros/jazzy/opt/gz_sim_vendor/lib/cmake/gz-sim8
)
# gz-plugin2 is found transitively by gz-sim8, but we need the register
# component explicitly for GZ_ADD_PLUGIN.
find_package(
    gz-plugin2
    REQUIRED
    COMPONENTS register
    HINTS /opt/ros/jazzy/opt/gz_plugin_vendor/lib/cmake/gz-plugin2
)

# ---------- FiveBarClosurePlugin ----------
add_library(FiveBarClosurePlugin SHARED src/FiveBarClosurePlugin.cc)

target_include_directories(
    FiveBarClosurePlugin
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(
    FiveBarClosurePlugin
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2-register
)

install(
    TARGETS FiveBarClosurePlugin
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# ---------- launch / config files ----------
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

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
