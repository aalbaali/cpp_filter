cmake_minimum_required( VERSION 3.10)
project(kalman_filter)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
# find_package(manif REQUIRED)
add_executable(${PROJECT_NAME} kalman_filter.cpp)
# add_executable(${PROJECT_NAME} playground.cpp)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "../include/RandomVariable/cpp")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

# # Add manif include directories to the target
# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message("CMAKE_BUILD_TYPE: Debug mode\n")
  # Add debug flag
  target_compile_definitions(kalman_filter PUBLIC DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  message("CMAKE_BUILD_TYPE: Release mode")
endif()

# Set required C++ standard flag
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
