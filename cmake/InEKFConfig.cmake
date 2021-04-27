# Return if target already defined
if( TARGET InEKF)
    return()
endif()

# Get parent path (project_path)
# New in version 3.20: 
#   https://cmake.org/cmake/help/latest/command/cmake_path.html#command:cmake_path
if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.20)
    message(DEBUG "Using 'cmake_path' to extract parent directory")
    cmake_path( GET         CMAKE_CURRENT_LIST_DIR 
                PARENT_PATH CMAKE_PARENT_CURRENT_LIST_DIR)
else()                
    get_filename_component(CMAKE_PARENT_CURRENT_LIST_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
endif()
# Source files directories 
set(CMAKE_SRC_DIR "${CMAKE_PARENT_CURRENT_LIST_DIR}/src")
# Include files directories
set(CMAKE_INCLUDE_DIR "${CMAKE_PARENT_CURRENT_LIST_DIR}/include")

# Find the RandomVariable directory
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_PARENT_CURRENT_LIST_DIR}/extern/RandomVariable/cmake")
find_package(RandomVariable)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# Find the manif library
find_package(manif REQUIRED)
# Find the YAML-cpp library
find_package(yaml-cpp REQUIRED)

add_library(InEKF 
    SHARED 
    "${CMAKE_SRC_DIR}/inekf_se2.cpp"
    "${CMAKE_INCLUDE_DIR}/InEKF/inekf_se2.h"
    )

target_include_directories(
    InEKF
    PUBLIC
    "${CMAKE_INCLUDE_DIR}/InEKF"
)
target_link_libraries(
    InEKF
    PUBLIC
        yaml-cpp
        Eigen3::Eigen
        ${MANIF_INCLUDE_DIRS}
        RandomVariable::RandomVariable
)

set_target_properties(
    InEKF
    PROPERTIES
        CXX_STANDARD    11
    )

# Display message
if(NOT DEFINED InEKF_FIND_QUIETLY)
    message( STATUS "Found InEKF: ${CMAKE_SOURCE_DIR}/InEKF")
endif()