# In this directory
* An implementation of a left-invariant extended Kalman filter (L-InEKF) on *SE(2)*.
* The simple kinematic model is of a robot driving in plane, with control inputs being 
    1. angular velocity, and
    2. linear velocity.
* Position (GPS) measurements are provided as exteroceptive measurements (possibly at different rates than the control inputs).

# Using the executable
A `configy.yml` (including its path) is passed to the executable as an argument. This configuration file consists of filenames of the sensor data. For example,
1. `filename_prior` contains a prior on the first *SE(2)$ state.
2. `filename_gyro` contains rate gyro sensor measurements.
3. `filename_vel` contains linear velocity (resolved in the body frame) sensor measurements.
4. `filename_gps` contains the position (GPS) sensor measurements.
5. `filename_out` specifies the location of the output of the executable.
Note that all the measurements are assumed to be noisy.    

# Generating sensor data
The sensor measurements can be generated using executables from the `RandomVariable` package that is included in `extern/` directory of this repo.
The files can be generated using the `RandomVariable/MATLAB_txt_IO/3D_sim/generate_data_SE2.m` MATLAB script. This script uses a `.mat` file generated from *SE(3)* data generator; this data-generator is not publicly available. 