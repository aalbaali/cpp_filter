#include "inekf_se2.h"

std::vector< PoseEstimate> GetSe2InekfEstimates( 
        PoseEstimate meas_prior, 
        std::vector< MeasGyro> meas_gyro,
        std::vector< MeasVel> meas_vel,
        std::vector< MeasGps> meas_gps){

        // Number of poses
    const unsigned int K = meas_gyro.size();
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
    // P_hat_km1 (convert from [theta; pos] to [pos; theta])
    CovPose P_hat_km1 = CovThetaPosToCovPosTheta( X_hat[0].cov());

    // Filtering
    for( size_t k = 1; k < K; k++){
        // Store time steps
        X_hat[k].setTime( meas_vel[k].time());

        // Interoceptive measurements at k - 1
        //  Velocity
        auto u_v_km1 = meas_vel[k-1].mean();
        //  Gyro
        auto u_g_km1   = meas_gyro[k-1].mean();
        // Get the tangent coordinates (twist)
        double dt_km1 = dt_func( k);        
        LieAlg u_km1(
                u_v_km1[0], 
                u_v_km1[1], 
                u_g_km1[0]
                );
        // Covariance on process noise
        CovQ Q_km1 = CovQ::Zero();
        Q_km1.block<2, 2>(0,0) = meas_vel[k-1].cov();
        Q_km1.block<1, 1>(2,2) = meas_gyro[k-1].cov();

        // Get matrix at k - 1
        EPose Xkm1_mat = X_hat[k-1].mean();
        // Note: covariance is already updated from previous iteration

        // Set pose at previous estimate
        Pose Xkm1( Xkm1_mat(0, 2), Xkm1_mat(1, 2), Xkm1_mat(0, 0), Xkm1_mat(1, 0));

        // Predict
        Pose X_k = Xkm1.plus( dt_km1 * u_km1);
        // (LI) Jacobian w.r.t. state
        JacF_Xkm1 J_F_xkm1 = (-dt_km1 * u_km1).exp().adj();
        // (LI) Jacobian w.r.t. process noise (w_km1)
        JacF_wkm1 J_F_wkm1 = -dt_km1 * JacF_wkm1::Identity();

        // Compute covariance on X_k
        CovPose P_k =   J_F_xkm1 * P_hat_km1 * J_F_xkm1.transpose() + 
                        J_F_wkm1 * Q_km1    * J_F_wkm1.transpose();
        
        // Correction
        if( idx_gps < meas_gps.size() && (meas_gps[idx_gps].time() <= X_hat[k].time())){
            // Implement a correction
            CovGps R_k   = meas_gps[idx_gps].cov();
            EMeasGps y_k = meas_gps[idx_gps].mean();
            // Jacobian of measurement function w.r.t. state
            JacYgps_Xk H_k;
            // Jacobian of innovation w.r.t. measurement noise
            Eigen::Matrix2d M_k = X_k.rotation().transpose();
            H_k.topLeftCorner< 2, 2>() = - Eigen::Matrix2d::Identity();
            H_k.topRightCorner< 2, 1>().setZero();

            // Predicted measurement            
            EMeasGps y_check_k = X_k.translation();
            
            // Innovation
            EMeasGps z = X_k.rotation().transpose() * (y_k - y_check_k);

            // Compute S_k
            JacYgps_nk S_k = H_k * P_k * H_k.transpose() + 
                             M_k * R_k * M_k.transpose();

            // Ensure symmetry
            JacYgps_nk S_k_tmp = 0.5 * (S_k + S_k.transpose());
            S_k = S_k_tmp;
            // Compute Kalman gain
            Eigen::Matrix<double, dof_x, dof_gps> K_k = P_k * H_k.transpose() * S_k.inverse();

            // Update state estimate (xhat) using a LI error
            // X_k = X_k.plus( LieAlg( - K_k * z));
            X_k += LieAlg( - K_k * z);
            // Pose X_hat_k = X_k.plus( LieAlg( - K_k * z));            
            // X_k = X_hat_k;
            
            // Update covariance
            P_k = ((CovPose::Identity()) - K_k * H_k) * P_k 
                    * ((CovPose::Identity()) - K_k * H_k).transpose() + 
                    K_k * M_k * R_k * M_k.transpose() * K_k.transpose();
            // Ensure symmetry
            CovPose P_k_tmp = 0.5 * (P_k + P_k.transpose());
            P_k = P_k_tmp;

            idx_gps++;
        }

        // Ensure symmetry
        P_k = 0.5 * (P_k + P_k.transpose().eval());
        
        // Update covariance at "previous" time step
        P_hat_km1 = P_k;

        
        
        // // Ensure symmetry
        // P_k_th_r = 0.5 * ( P_k_th_r + P_k_th_r.transpose().eval());

        // Store estimates
        X_hat[k].setMean( X_k.transform());
        // Store the covariance in the appropriate format [th; pos]
        // Note: switch covariance type from covariance on [position; theta] to [theta; position] (to do the analysis)
        X_hat[k].setCov( CovPosThetaToCovThetaPos( P_k));
        X_hat[k].setCovIsGlobal( false);
    }   

    return X_hat;
}

std::vector< PoseEstimate> GetSe2InekfEstimates( const std::string filename_config){
    // @param[in] filename_config 
    //      Filename (including path) of the .yml configuration file.
    // @return std::vector of PoseEstimate (instance of RandomVariable class).
    // Assumptions:
    //      - Assumes the files include GPS measurements as exteroceptive measurements.

    // Read .yml configuration file
    YAML::Node config= YAML::LoadFile( filename_config);
#ifndef NDEBUG
    // Display message if in debug model
    std::cout << "Reading .yml configuration from '" << filename_config << "'" << std::endl;        
#endif

    // Read sensor files
    // Prior
    const std::string filename_prior = config["filename_prior"].as<std::string>();
    //  Gyro
    const std::string filename_gyro  = config["filename_gyro"].as<std::string>();
    //  Velocity
    const std::string filename_vel   = config["filename_vel"].as<std::string>();
    //  GPS
    const std::string filename_gps   = config["filename_gps"].as<std::string>();

    // Import data
    //  Prior
    PoseEstimate meas_prior = RV::IO::import< PoseEstimate>( filename_prior)[0];
    //  Gyro
    std::vector< MeasGyro> meas_gyro      = RV::IO::import< MeasGyro>( filename_gyro);
    //  Velocity
    std::vector< MeasVel> meas_vel        = RV::IO::import< MeasVel>( filename_vel);
    //  GPS
    std::vector< MeasGps> meas_gps        = RV::IO::import< MeasGps>( filename_gps);
    
    return GetSe2InekfEstimates(
                meas_prior,
                meas_gyro,
                meas_vel,
                meas_gps
            );    
}

CovPose CovPosThetaToCovThetaPos( CovPose P_pos_th){
        CovPose P_th_r;
        P_th_r( 0, 0)                     = P_pos_th( 2, 2);
        //  X-cov pos-heading
        P_th_r.bottomLeftCorner< 2, 1>()  = P_pos_th.topRightCorner< 2, 1>();
        //  X-cov heading-pos
        P_th_r.topRightCorner< 1, 2>()    = P_pos_th.bottomLeftCorner< 1, 2>();
        //  Cov pos
        P_th_r.bottomRightCorner<2, 2>()  = P_pos_th.topLeftCorner< 2, 2>();    

        return P_th_r;
}

CovPose CovThetaPosToCovPosTheta( CovPose P_th_pos){
        CovPose P_pos_th;
        P_pos_th( 2, 2)                     = P_th_pos( 0, 0);
        //  X-cov pos-heading
        P_pos_th.topRightCorner< 2, 1>()  = P_th_pos.bottomLeftCorner< 2, 1>();
        //  X-cov heading-pos
        P_pos_th.bottomLeftCorner< 1, 2>() = P_th_pos.topRightCorner< 1, 2>();
        //  Cov pos
        P_pos_th.topLeftCorner< 2, 2>()  = P_th_pos.bottomRightCorner<2, 2>();    

        return P_pos_th;
}
