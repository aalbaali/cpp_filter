#include <iostream>
#include "inekf_se2.h"

int main(int argc, const char* argv[]){
    // Configurations
    YAML::Node config;
    std::string filename_config;
    filename_config = argv[1];
    std::cout << argc << " arguments passed: " << filename_config << std::endl;
    config = YAML::LoadFile( filename_config);
    
    // Get output file name
    const std::string filename_kf   = config["filename_kf"].as<std::string>();

    // Run L-InEKF
    std::vector< PoseEstimate> X_hat = GetSe2InekfEstimates( filename_config);

    RV::IO::write( X_hat, filename_kf, "X");
}
