cmake_minimum_required( VERSION 3.10)
project(example_SE2)

add_executable(example_SE2 example_SE2.cpp)
# add_executable(${PROJECT_NAME} playground.cpp)

target_link_libraries(example_SE2 
  yaml-cpp
  RandomVariable::RandomVariable
  )

target_include_directories(example_SE2 PRIVATE ${EIGEN3_INCLUDE_DIRS})


# Add manif include directories to the target
target_include_directories(example_SE2 SYSTEM PUBLIC ${manif_INCLUDE_DIRS})


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message("CMAKE_BUILD_TYPE: Debug mode\n")
  # Add debug flag
  target_compile_definitions(${PROJECT_NAME} PUBLIC DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  message("CMAKE_BUILD_TYPE: Release mode")
endif()

# Set required C++ standard flag
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
