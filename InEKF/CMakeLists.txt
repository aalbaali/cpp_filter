cmake_minimum_required( VERSION 3.10)
project(example_SE2)

list(APPEND CMAKE_PREFIX_PATH "../extern/RandomVariable/cmake")
find_package(RandomVariable)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
find_package(manif REQUIRED)

add_executable(example_SE2 example_SE2.cpp inekf_se2.cpp inekf_se2.h)
# add_executable(${PROJECT_NAME} playground.cpp)

target_link_libraries(example_SE2 
  yaml-cpp
  RandomVariable::RandomVariable
  )

target_include_directories(example_SE2 PRIVATE ${EIGEN3_INCLUDE_DIRS})


# Add manif include directories to the target
target_include_directories(example_SE2 SYSTEM PUBLIC ${manif_INCLUDE_DIRS})


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Add debug flag
  target_compile_definitions(example_SE2 PUBLIC DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
endif()

# Set required C++ standard flag
set_property(TARGET example_SE2 PROPERTY CXX_STANDARD 17)
