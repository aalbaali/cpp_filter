cmake_minimum_required( VERSION 3.10)
project(example_SE2)

list(APPEND CMAKE_PREFIX_PATH "../extern/RandomVariable/cmake")
find_package(RandomVariable)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
find_package(manif REQUIRED)

add_library(inekf_se2 SHARED inekf_se2.cpp inekf_se2.h)
target_link_libraries(inekf_se2 
  PUBLIC
    yaml-cpp
    Eigen3::Eigen
    ${MANIF_INCLUDE_DIRS}
    RandomVariable::RandomVariable
  )

add_executable(example_SE2 example_SE2.cpp)

target_link_libraries(example_SE2   
  PRIVATE
    yaml-cpp
    inekf_se2
  )


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Add debug flag
  target_compile_definitions(example_SE2 PRIVATE DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
endif()

# Set required C++ standard flag
set_property(TARGET example_SE2 PROPERTY CXX_STANDARD 17)
