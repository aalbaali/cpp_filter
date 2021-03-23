// Degrees of freedom of pose (SE(2))
const size_t dof_x = 3;
// Embedded dimension (SE(2) -> robot lives in 2D)
const size_t dim_x = 2;
const size_t dof_gyro = 1;  // DOF of gyro measurement
const size_t dof_vel  = 1;  // DOF of velocity measurement
const size_t dof_gps  = 2;  // DOF of GPS measurement

// Eigen matrix types
// typdef Eigen::Matrix< double, dof_x

// Random variables
typedef RV::RandomVariable< dof_gyro> MeasGyro;
typedef RV::RandomVariable< dof_gyro> MeasVel;
typedef RV::RandomVariable< dof_gps > MeasGps;

// Filenames
//  Gyro
const std::string filename_gyro = "/home/aalbaali/Documents/Data/Data_generator/SE2/meas_gyro.txt";
//  Velocity
const std::string filename_vel  = "/home/aalbaali/Documents/Data/Data_generator/SE2/meas_vel.txt";
//  GPS
const std::string filename_gps  = "/home/aalbaali/Documents/Data/Data_generator/SE2/meas_gps.txt";
