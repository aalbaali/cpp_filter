cmake_minimum_required(VERSION 3.10.0)
project(CppFilter VERSION 0.1.0)

include(CTest)
enable_testing()

# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
find_package(manif REQUIRED)

# Add the RandomVariable header class
add_subdirectory( extern/RandomVariable)
# target_include_directories(${PROJECT_NAME} PUBLIC extern/RandomVariable/include)
# Add InEKF
add_subdirectory( InEKF)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

# install(TARGETS libs_example DESTINATION bin)