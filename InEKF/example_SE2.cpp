#include "config.h"

// // Process model function
// Pose process_model( Pose X_km1, LieAlg u_km1, double dt){
//     return X_km1 + (dt * u_km1);
// }
// // Jacobian


int main(){
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
    
    // Dead-reckoning
    for( size_t k = 1; k < K; k++){
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
        // Jacobian w.r.t. state
        JacF_Xkm1 J_F_xkm1;
        // Jacobian w.r.t. process noise (w_km1)
        JacF_wkm1 J_F_wkm1;

        // Motion/process model. Get the Jacobians as well
        Pose Xk = Xkm1.plus( u_km1, J_F_xkm1, J_F_wkm1);
        
        // Compute covariance on X_k
        CovPose Cov_Xk =   J_F_xkm1 * Cov_Xkm1 * J_F_xkm1.transpose() + 
                        J_F_wkm1 * Q_km1    * J_F_wkm1.transpose();
        
        // Store estimates
        X_hat[k].setMean( Xk.transform());
        X_hat[k].setCov( Cov_Xk);
        X_hat[k].setCovIsGlobal( false);

        // Store time steps
        X_hat[k].setTime( meas_vel[k-1].time());
    }

    
    RV::IO::write( X_hat, filename_out, "X");
}