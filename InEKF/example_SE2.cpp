#include "config.h"

int main(){
    // Import data
    //  Gyro
    std::vector< MeasGyro> meas_gyro = RV::IO::import< MeasGyro>( filename_gyro);
    //  Velocity
    std::vector< MeasVel> meas_vel   = RV::IO::import< MeasVel>( filename_vel);
    //  GPS
    std::vector< MeasGps> meas_gps   = RV::IO::import< MeasGps>( filename_gps);
    
    // Number of poses
    const unsigned int num_poses = meas_gyro.size() + 1;
    // Lambda function that extracts the sample time (dt)
    auto dt_func = [&meas_gyro](int k){
        return meas_gyro[k].time() - meas_gyro[k-1].time();
    };

    for(size_t i = 0; i < num_poses - 1; ++i){
        std::cout << meas_gyro[i].mean().transpose() << "\t";
        std::cout << meas_vel[i].mean().transpose() << std::endl;
    }
    // Vector of pose estimates
    std::vector< Pose> Xhat( num_poses);

    // Dead-reckoning
    // Initialize first pose
    Xhat[0].setIdentity();
    for( size_t k = 1; k < num_poses; k++){
        // Interoceptive measurements at k - 1
        //  Velocity
        auto u_v_km1 = meas_vel[k-1].mean();
        //  Gyro
        auto u_g_km1   = meas_gyro[k-1].mean();
        // Get the tangent coordinates (twist)
        double dt_km1 = dt_func( k);        
        manif::SE2Tangentd u_km1(
                dt_km1 * u_v_km1[0], 
                dt_km1 * u_v_km1[1], 
                dt_km1 * u_g_km1[0]
        );
        Xhat[k] = Xhat[k-1] + u_km1;
    }

    // Display dead reckoning solution
    for( auto Xk : Xhat){
        std::cout << Xk.transform() << std::endl;
    }

    // // Export Xhat to a RandomVariable object (temporary solution)
    std::vector< PoseEstimate> estimated_states_rv( num_poses);
    for( size_t i = 0; i < num_poses; ++i){
        estimated_states_rv[i] = PoseEstimate( Xhat[i].transform());
    }
    RV::IO::write( estimated_states_rv, filename_out, "X");
}