project(kalman_filter)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
# find_package(manif REQUIRED)
add_executable(${PROJECT_NAME} kalman_filter.cpp)

# Include the parser
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "include/data_generator/cpp_parser/")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

# # Add manif include directories to the target
# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})


# Add debug flag
target_compile_definitions(kalman_filter PUBLIC DEBUG)

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${CXX_11_EXAMPLE_TARGETS} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${CXX_11_EXAMPLE_TARGETS} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${CXX_11_EXAMPLE_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${CXX_11_EXAMPLE_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
endif()
