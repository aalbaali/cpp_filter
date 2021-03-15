#include <iostream>
#include <vector>
#include <iomanip> // For nice outputs
#include <unsupported/Eigen/MatrixFunctions>
#include "Eigen/Dense"

// Include parser
#include "data_parser.h"

#include "Measurement.h"

// Dimension of control input measurements
const size_t size_u = 1;
// Dimension of the exeteroceptive measurement
const size_t size_y = 1;

// Acceleration measurement class
typedef Measurement< size_u> MeasControlInput;
typedef Measurement< size_y> MeasGPS;

// Control input file name
const std::string file_name_u = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/msd_acc.txt";

const std::string file_name_gps = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/msd_pos.txt";


// Logging the data
#include "Exporter.h"

int main(){

    // ************************************************
    // Import control input
    std::vector< MeasControlInput> meas_control_input = ImportMeasurementsObjectVector<MeasControlInput>( file_name_u);

    // ************************************************
    // Import GPS measurements
    std::vector< MeasGPS> meas_gps = ImportMeasurementsObjectVector<MeasGPS>( file_name_gps);

    const std::string file_name_out = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/dd.txt";

    // Create a vector of strings for header
    std::vector<std::string> header( size_y * (size_y + 1) + 1);
    header[0] = "Time";
    header[1] = "y1";
    header[2] = "var_y1";


    RandomVariable::LogMeasurements( meas_gps, header, file_name_out);

    // // Display measurements
    // std::cout << "\n\n===============================\nControl Input\n" << std::endl;
    // std::cout << "Time\t\tMeas\t\tVar" << std::endl;
    // for( auto meas : meas_control_input){
    // // for( size_t i; i < 10; i++){
    // //     auto meas = meas_control_input[i];
    //     std::cout << std::setw(1) << meas.time() << "\t\t";
    //     std::cout << std::setw(2) << meas.meas().transpose() << "\t\t";
    //     std::cout << std::setw(5) << meas.cov() << std::endl;
    // }
    // std::cout << "\n\n===============================\nGPS Meas\n" << std::endl;
    // std::cout << "Time\t\tMeas\t\tVar" << std::endl;
    // for( auto meas : meas_gps){
    //     std::cout << std::setw(1) << meas.time() << "\t\t";
    //     std::cout << std::setw(2) << meas.meas().transpose() << "\t\t";
    //     std::cout << std::setw(5) << meas.cov() << std::endl;
    // }
}