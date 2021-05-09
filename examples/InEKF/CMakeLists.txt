cmake_minimum_required( VERSION 3.10)
project(example_SE2)

# Append path with Config.cmake dirs/files

# Find the RandomVariable directory
find_package(RandomVariable)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# Find the manif library
find_package(manif REQUIRED)
# Find the YAML-cpp library
find_package(yaml-cpp REQUIRED)
find_package(InEKF)

add_executable(example_SE2 example_SE2.cpp)

target_link_libraries(example_SE2   
  PUBLIC    
    InEKF::SE2
  )


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Add debug flag
  target_compile_definitions(example_SE2 PRIVATE DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
endif()

# Set required C++ standard flag
set_property(TARGET example_SE2 PROPERTY CXX_STANDARD 17)
