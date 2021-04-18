#include "config.h"

int main(int argc, const char* argv[]){
    // Configurations
    YAML::Node config;
    if(argc > 1){
        std::string filename_config = argv[1];
        std::cout << argc << " arguments passed: " << filename_config << std::endl;
        config = YAML::LoadFile( filename_config);
    }else{
        // read yaml file with config settings
        std::string filename_config = "../config";
        std::cout << "Reading .yml configuration from '" << filename_config << "'" << std::endl;        
        config = YAML::LoadFile( filename_config);
    }    

    // Prior
    const std::string filename_prior = config["filename_prior"].as<std::string>();
    //  Gyro
    const std::string filename_gyro  = config["filename_gyro"].as<std::string>();
    //  Velocity
    const std::string filename_vel   = config["filename_vel"].as<std::string>();
    //  GPS
    const std::string filename_gps   = config["filename_gps"].as<std::string>();
    // Estimated states
    const std::string filename_out       std::string= config["filename_out"].as<std::string>();
    // If there's an argument, then read the YAML configuration file from input. Otherwise, use default directory

    // Import data
    //  Prior
    PoseEstimate meas_prior = RV::IO::import< PoseEstimate>( filename_prior)[0];
    //  Gyro
    std::vector< MeasGyro> meas_gyro      = RV::IO::import< MeasGyro>( filename_gyro);
    //  Velocity
    std::vector< MeasVel> meas_vel        = RV::IO::import< MeasVel>( filename_vel);
    //  GPS
    std::vector< MeasGps> meas_gps        = RV::IO::import< MeasGps>( filename_gps);
    
    // Number of poses
    const unsigned int K = meas_gyro.size() + 1;
    // Lambda function that extracts the sample time (dt)
    auto dt_func = [&meas_gyro](int k){
        return meas_gyro[k].time() - meas_gyro[k-1].time();
    };
    
    // Vector of estimated poses
    std::vector< PoseEstimate> X_hat( K);

    // Make sure meas_prior is a valid measurement (using manif)
    {
        // Eigen matrix
        EPose X0_mat = meas_prior.mean();
        // Manif
        Pose X0_man( X0_mat(0, 2), X0_mat(1, 2), atan2( X0_mat(1, 0), X0_mat(0, 0)));
        // Normalize
        X0_man.normalize();
        // Export the SE2 object
        meas_prior.setMean( X0_man.transform());
    }
    // Set initial estimate
    X_hat[0] = meas_prior;
    
    // Index that keeps track of the gps measurements
    size_t idx_gps = 0;
    // Filtering
    for( size_t k = 1; k < K; k++){
        // Store time steps
        X_hat[k].setTime( meas_vel[k-1].time());

        // Interoceptive measurements at k - 1
        //  Velocity
        auto u_v_km1 = meas_vel[k-1].mean();
        //  Gyro
        auto u_g_km1   = meas_gyro[k-1].mean();
        // Get the tangent coordinates (twist)
        double dt_km1 = dt_func( k);        
        LieAlg u_km1(
                dt_km1 * u_v_km1[0], 
                dt_km1 * u_v_km1[1], 
                dt_km1 * u_g_km1[0]
                );
        // Covariance on process noise
        CovQ Q_km1 = CovQ::Zero();
        Q_km1.block<2, 2>(0,0) = meas_vel[k-1].cov();
        Q_km1.block<1, 1>(2,2) = meas_gyro[k-1].cov();

        // Get matrix at k - 1
        EPose Xkm1_mat = X_hat[k-1].mean();
        // Covariance at k - 1
        CovPose Cov_Xkm1 = X_hat[k-1].cov();

        // Set pose at previous estimate
        Pose Xkm1( Xkm1_mat(0, 2), Xkm1_mat(1, 2), Xkm1_mat(0, 0), Xkm1_mat(1, 0));

        // Predict
        Pose X_k = Xkm1.plus( u_km1);
        // (LI) Jacobian w.r.t. state
        JacF_Xkm1 J_F_xkm1 = (-u_km1).exp().adj();
        // (LI) Jacobian w.r.t. process noise (w_km1)
        JacF_wkm1 J_F_wkm1 = -dt_km1 * JacF_wkm1::Identity();

        // Compute covariance on X_k
        CovPose P_k =   J_F_xkm1 * Cov_Xkm1 * J_F_xkm1.transpose() + 
                        J_F_wkm1 * Q_km1    * J_F_wkm1.transpose();
        
        // Correction
        if( idx_gps < meas_gps.size() && (meas_gps[idx_gps].time() <= X_hat[k].time())){
            // Implement a correction
            auto R_k = meas_gps[idx_gps].cov();
            auto y_k = meas_gps[idx_gps].mean();
            // Jacobian of measurement function w.r.t. state
            JacYgps_Xk H_k;
            // Jacobian of innovation w.r.t. measurement noise
            Eigen::Matrix2d M_k = X_k.rotation().transpose();
            H_k.topLeftCorner< 2, 2>() = - Eigen::Matrix2d::Identity();
            H_k.topRightCorner< 2, 1>().setZero();

            // Predicted measurement            
            auto y_check_k = X_k.translation();
            
            // Innovation
            auto z = X_k.rotation().transpose() * (y_k - y_check_k);

            // Compute S_k
            JacYgps_nk S_k = H_k * P_k * H_k.transpose() + 
                             M_k * R_k * M_k.transpose();

            // Ensure symmetry
            S_k = 0.5 * (S_k + S_k.transpose());
            // Compute Kalman gain
            Eigen::Matrix<double, dof_x, dof_gps> K_k = P_k * H_k.transpose() * S_k.inverse();

            // Update state estimate (xhat) using a LI error
            X_k = X_k + (LieAlg( - K_k * z));
            // Update covariance
            P_k = ((CovPose::Identity()) - K_k * H_k) * P_k 
                    * ((CovPose::Identity()) - K_k * H_k).transpose() + 
                    K_k * M_k * R_k * M_k.transpose() * K_k.transpose();
            // Ensure symmetry
            P_k = 0.5 * (P_k + P_k.transpose());

            idx_gps++;
        }

        // Store estimates
        X_hat[k].setMean( X_k.transform());
        X_hat[k].setCov( P_k);
        X_hat[k].setCovIsGlobal( false);
    }

    RV::IO::write( X_hat, filename_out, "X");
}