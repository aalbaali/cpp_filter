#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Dense"

// My custom RandomVariable class
#include "RV.h"

#include "config.h"

int main(){
    // Import data
    //  Gyro
    std::vector< MeasGyro> meas_gyro = RV::IO::import< MeasGyro>( filename_gyro);
    //  Velocity
    std::vector< MeasVel> meas_vel   = RV::IO::import< MeasVel>( filename_vel);
    //  GPS
    std::vector< MeasGps> meas_gps   = RV::IO::import< MeasGps>( filename_gps);
}