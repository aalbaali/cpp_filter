cmake_minimum_required( VERSION 3.10)
project(example_kf)

# Find the RandomVariable directory
list(APPEND CMAKE_PREFIX_PATH "../../extern/RandomVariable/cmake")
find_package(RandomVariable)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
# find_package(manif REQUIRED)
add_executable(example_kf kalman_filter.cpp)

target_link_libraries(example_kf 
  RandomVariable::RandomVariable
  )

target_include_directories(example_kf SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message("CMAKE_BUILD_TYPE: Debug mode\n")
  # Add debug flag
  target_compile_definitions(example_kf PUBLIC DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  message("CMAKE_BUILD_TYPE: Release mode")
endif()

# Set required C++ standard flag
set_property(TARGET example_kf PROPERTY CXX_STANDARD 17)
