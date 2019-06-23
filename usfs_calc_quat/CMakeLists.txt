cmake_minimum_required(VERSION 2.8.3)
project(usfs_calc_quat)

add_executable(usfs_calc_quat usfs_calc_quat.cpp)
target_link_libraries(
    usfs_calc_quat
    imu_filter
)
