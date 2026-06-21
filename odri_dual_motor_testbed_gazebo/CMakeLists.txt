cmake_minimum_required(VERSION 3.22)
project(odri_dual_motor_testbed_gazebo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# gz-sim8 (Gazebo Harmonic) – vendored under the Jazzy distribution
find_package(gz-sim8 REQUIRED HINTS
             /opt/ros/jazzy/opt/gz_sim_vendor/lib/cmake/gz-sim8)
# gz-plugin2 is found transitively by gz-sim8, but we need the register
# component explicitly for GZ_ADD_PLUGIN.
find_package(
  gz-plugin2 REQUIRED
  COMPONENTS register HINTS
             /opt/ros/jazzy/opt/gz_plugin_vendor/lib/cmake/gz-plugin2)

# ---------- FiveBarClosurePlugin ----------
add_library(FiveBarClosurePlugin SHARED src/FiveBarClosurePlugin.cc)

target_include_directories(
  FiveBarClosurePlugin
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

target_link_libraries(FiveBarClosurePlugin gz-sim8::gz-sim8
                      gz-plugin2::gz-plugin2-register)

install(
  TARGETS FiveBarClosurePlugin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---------- launch / config files ----------
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
