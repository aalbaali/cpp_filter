cmake_minimum_required(VERSION 3.10.0)
project(CppFilter VERSION 0.1.0)

include(CTest)
enable_testing()

# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
find_package(manif REQUIRED)


# Add the RandomVariable header class
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/extern/RandomVariable/cmake")
find_package(RandomVariable)

# Add Kalman fitler example
message(STATUS "Building KalmanFilter")
add_subdirectory( KalmanFilter)
# Add InEKF
message(STATUS "Building InEKF")
add_subdirectory( InEKF)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

# install(TARGETS libs_example DESTINATION bin)