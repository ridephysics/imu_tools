cmake_minimum_required(VERSION 2.8.3)
project(imu_tools)

add_compile_options(
    -Wall
    -Wextra
    -Werror
)

add_subdirectory(imu_filter_madgwick)
