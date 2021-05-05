
# Find the Eigen library
find_package(Eigen3 REQUIRED)
# # Find the manif library
find_package(manif REQUIRED)

# Add the RandomVariable header class
find_package(RandomVariable)

# Build the InEKF library
add_library(SE2 
    STATIC  
        "${CMAKE_CURRENT_LIST_DIR}/inekf_se2.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/inekf_se2.h"
)

# Create alias
add_library(InEKF::SE2 ALIAS SE2)

target_include_directories(
    SE2
    PUBLIC
        ${EIGEN_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/InEKF>"
)

target_link_libraries(
    SE2
    PUBLIC
        yaml-cpp
        Eigen3::Eigen
        ${MANIF_LIBRARY}
        RandomVariable::RandomVariable
)

set_target_properties(
    SE2
    PROPERTIES
        CXX_STANDARD    11
    )


# install the target and create export-set
install(TARGETS SE2
        EXPORT InEKFTargets
        LIBRARY DESTINATION  "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION  "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION  "${CMAKE_INSTALL_BINDIR}"
        INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/InEKF"
)

# install header file
install(FILES 
        "inekf_se2.h"
        DESTINATION
         "${CMAKE_INSTALL_INCLUDEDIR}/InEKF"
)


