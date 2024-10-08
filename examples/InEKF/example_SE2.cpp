#include "inekf_se2.h"
#include <iostream>

int main(int argc, const char *argv[]) {
  // Configurations
  YAML::Node config;
  std::string filename_config;
  filename_config = argv[1];
  std::cout << "Filename passed: " << filename_config << std::endl;
  config = YAML::LoadFile(filename_config);

  // Get output file name
  const std::string filename_kf = config["filename_kf"].as<std::string>();

  // Run L-InEKF
  std::vector<PoseEstimate> X_hat = GetSe2InekfEstimates(filename_config);

  std::cout << "Exporting L-InEKF estimates to "
            << config["filename_kf"].as<std::string>() << std::endl;
  RV::IO::write(X_hat, filename_kf, "X");
}
