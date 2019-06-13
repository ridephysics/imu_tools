cmake_minimum_required(VERSION 2.8.3)
project(imu_filter_madgwick)

option(ENABLE_TESTING "compile tests" OFF)

include(FindPkgConfig)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
find_package(PkgConfig)


# create imu_filter library
add_library (imu_filter src/imu_filter.cpp src/stateless_orientation.cpp)
target_link_libraries(
    imu_filter
    Qt5::Core
    Qt5::Gui
)
target_include_directories(imu_filter PUBLIC
  include
)

if(ENABLE_TESTING)
  enable_testing()
  include(GoogleTest)

  add_executable(${PROJECT_NAME}-madgwick_test
    test/stateless_orientation_test.cpp
    test/madgwick_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}-madgwick_test
    imu_filter
    gtest
  )

  gtest_discover_tests(${PROJECT_NAME}-madgwick_test)
endif()
