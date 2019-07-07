cmake_minimum_required(VERSION 2.8.3)
project(imu_complementary_filter)

## Declare a cpp library
add_library(complementary_filter
  src/complementary_filter.cpp
  include/imu_complementary_filter/complementary_filter.h
)

target_include_directories(complementary_filter PUBLIC
  include
)
