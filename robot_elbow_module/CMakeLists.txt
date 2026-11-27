cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
# PlatformIO adds the __pio_env component for its generated build files.
# Keep the firmware build minimal, but do not exclude that required component.
set(COMPONENTS main __pio_env)
project(robot_elbow_module)

