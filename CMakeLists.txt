cmake_minimum_required(VERSION 3.15)
project(InEKF VERSION 0.1)

# make cache variables for install destinations
include(GNUInstallDirs)

include(CTest)
enable_testing()

# SE(2) InEKF library
add_subdirectory(InEKF)

if(BUILD_EXAMPLES)
    # Add examples subdirectory
    message(STATUS "Building examples")
    add_subdirectory( examples)
endif()

########################################
# Configuration
########################################
# include CMakePackageConfigHelpers macro
include(CMakePackageConfigHelpers)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/InEKFConfigVersion.cmake"
  VERSION ${CMAKE_PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/InEKFConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/InEKF
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# install config files
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/InEKFConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/InEKFConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/InEKF
)