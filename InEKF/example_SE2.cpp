#include <iostream>
#include "inekf_se2.h"
// #include <string>
// #include <vector>

// // Eigen
// #include "Eigen/Dense"

// // Manif
// #include "manif/SE2.h"

// // YAML C++
// #include "yaml-cpp/yaml.h"

// // My custom RandomVariable class
// #include "RV.h"
// #include "RVIO.h"


// // Degrees of freedom of pose (SE(2))
// const size_t dof_x = 3;
// // Embedded dimension (SE(2) -> robot lives in 2D)
// const size_t dim_x = 2;
// const size_t dof_gyro = 1;  // DOF of gyro measurement
// const size_t dof_vel  = 2;  // DOF of velocity measurement
// const size_t dof_gps  = 2;  // DOF of GPS measurement

// // Random variables
// typedef RandomVariable< dof_gyro>    MeasGyro;
// typedef RandomVariable< dof_vel >    MeasVel;
// typedef RandomVariable< dof_gps >    MeasGps;
// typedef RandomVariable< 3, 3, dof_x> PoseEstimate;

// // Lie Group classes
// typedef manif::SE2d         Pose;       // Pose variable
// typedef manif::SE2Tangentd  LieAlg;     // Tangent vectors

// // Eigen matrix sizes
// // SE2 matrix
// typedef Eigen::Matrix< double, 3, 3>         EPose;
// //  Covariances
// typedef Eigen::Matrix< double, dof_x, dof_x> CovPose;
// // Process model
// //  Jacobian w.r.t. previous state
// typedef Eigen::Matrix< double, 3, 3>                    JacF_Xkm1;
// //  Jacobian w.r.t. noise
// typedef Eigen::Matrix< double, 3, dof_gyro + dof_vel>   JacF_wkm1;
// //  Covariance on process noise
// typedef Eigen::Matrix< double, dof_gyro + dof_vel, dof_gyro + dof_vel> CovQ;
// // GPS measurement model
// //  Jacobian w.r.t. state 
// typedef Eigen::Matrix< double, dof_gps, dof_x>          JacYgps_Xk;
// //  Jacobian w.r.t. measurement noise 
// typedef Eigen::Matrix< double, dof_gps, dof_gps>        JacYgps_nk;


// Function that takes the .yml filename (including path) and returns the InEKF estiamtes.
std::vector< PoseEstimate> GetSe2InekfEstimates( const std::string filename_config);

int main(int argc, const char* argv[]){
    // Configurations
    YAML::Node config;
    std::string filename_config;
    if(argc > 1){
        filename_config = argv[1];
        std::cout << argc << " arguments passed: " << filename_config << std::endl;
        config = YAML::LoadFile( filename_config);
    }else{
        // read yaml file with config settings
        filename_config = "../config.yml";
        std::cout << "Reading .yml configuration from '" << filename_config << "'" << std::endl;        
        config = YAML::LoadFile( filename_config);
    }    
    // Get output file name
    const std::string filename_out   = config["filename_out"].as<std::string>();

    // Run L-InEKF
    std::vector< PoseEstimate> X_hat = GetSe2InekfEstimates( filename_config);

    RV::IO::write( X_hat, filename_out, "X");
}
