cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32c3")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# LIMB-HT25 enabled minimal builds so unrelated Bluetooth/network components
# were not compiled into small sensor targets.
idf_build_set_property(MINIMAL_BUILD ON)
project(aurora_dual_imu_serial)
