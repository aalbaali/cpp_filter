#include <iostream>
#include <vector>
#include <iomanip> // For nice outputs (setw)
#include <string>  // To definte the file_name for exporting data
#include <sstream>
#include <tuple>

#include <unsupported/Eigen/MatrixFunctions>
#include "Eigen/Dense"

// Include parser
#include "RV.h"

Eigen::VectorXd process_model( Eigen::VectorXd x_km1, Eigen::VectorXd u_km1, Eigen::MatrixXd A, Eigen::MatrixXd B){
    return A * x_km1 + B * u_km1;
}

// Dimension of control input measurements
const size_t size_u = 1;
// Dimension of the exeteroceptive measurement
const size_t size_y = 1;
// Dimension of each pose
const size_t size_x = 2;

// Eigen matrix types
typedef Eigen::Matrix< double, size_x, 1>       VectorPose;
typedef Eigen::Matrix< double, size_x, size_x>  CovPose;

typedef Eigen::Matrix< double, size_x, size_x>  MatrixA;
typedef Eigen::Matrix< double, size_x, size_x>  MatrixQ;
typedef Eigen::Matrix< double, size_x, size_u>  MatrixB;


// Acceleration measurement class
typedef RV::RandomVariable< size_u> MeasControlInput;
typedef RV::RandomVariable< size_y> MeasGPS;
typedef RV::RandomVariable< size_x> PoseEstimate;

// Returns A, B, Q discrete-time matrices
template<typename T_L, typename T_Qct>
std::tuple<MatrixA, MatrixB, MatrixQ> getDiscreteABQ(MatrixA A_ct, MatrixB B_ct, T_L L_ct, T_Qct Q_ct, double dt){
    // Construct the Xi matrix
    Eigen::Matrix<double, 3 * size_x + size_u, 3 * size_x + size_u> Xi;
    Xi.setZero();
    Xi.block< size_x, size_x>(0, 0)                 =  A_ct;
    Xi.block< size_x, size_x>(size_x, size_x)         = -A_ct.transpose();
    Xi.block< size_x, size_x>(2 * size_x, 2 * size_x) =  A_ct;
    Xi.block< size_x, size_u>(2 * size_x, 3 * size_x) =  B_ct;
    Xi.block< size_x, size_x>(0, size_x)             =  L_ct * Q_ct * L_ct.transpose();

    // Eigen::Matrix<double, 3 * size_x + size_u, 3 * size_x + size_u>
    auto
        Gamma = (dt * Xi).exp();
    // Extract the blocks
    MatrixA A_dt = Gamma.block<size_x, size_x> (0, 0);
    MatrixB B_dt = Gamma.block<size_x, size_u> (2 * size_x, 3 * size_x);
    auto Gamma_12 = Gamma.block<size_x, size_x>(0, size_x);
    MatrixQ Q_dt = Gamma_12 * A_dt.transpose();    
    return std::make_tuple(A_dt, B_dt, Q_dt);
}



// Control input file name
const std::string file_name_u = "/home/aalbaali/Documents/Data/Data_generator/linear_system/msd_acc.txt";

const std::string file_name_gps = "/home/aalbaali/Documents/Data/Data_generator/linear_system/msd_pos.txt";


int main(){
    // GPS sensor
    Eigen::Matrix<double, size_y, size_x> sys_C;
    sys_C << 1, 0;
    Eigen::Matrix<double, size_y, size_y> sys_M;
    sys_M.setIdentity();

    // Define CT matrices
    // *********************************************
    // Cotinuous-time (CT) system parameters
    // Continuous-time system matrix
    MatrixA sys_A_ct;
    // Continuous-time control matrix
    MatrixB sys_B_ct;
    Eigen::Vector2d sys_L_ct;
    // System is a mass-spring-damper system with unit mass, spring-constant, and damping.
    sys_A_ct << 0, 1, -1, -1;
    sys_B_ct << 0, 1;
    sys_L_ct = sys_B_ct;

// #ifdef DEBUG
//     std::cout << "sys_A_dt:\n" << sys_A << std::endl;
//     std::cout << "sys_B_dt:\n" << sys_B << std::endl;
// #endif

    // *********************************************
    // Initial conditions
    VectorPose x_0( 0., 0.);
    // Covariance on initial condition
    CovPose cov_P_0;
    cov_P_0 << 1e-3, 0, 0, 1e-3;


    // ************************************************
    // Import control input
    std::vector< MeasControlInput> meas_control_input = RV::IO::import<MeasControlInput>( file_name_u);
    // Lambda function that extractes the sample time (dt)
    auto dt_func = [&meas_control_input](int k){
        return meas_control_input[k].time() - meas_control_input[k-1].time();
    };
#ifdef DEBUG
    // Try it out
    std::cout << "dt_{5}: " << dt_func(5) << std::endl;
#endif

    // ************************************************
    // Import GPS measurements
    std::vector< MeasGPS> meas_gps = RV::IO::import<MeasGPS>( file_name_gps);

    // *********************************************
    //  Number of poses 
    // TODO: change it to match the number of control input measurements
    const unsigned int K = meas_control_input.size() + 1;
    // const unsigned int K = 10;

    // *********************************************
    //  Estaimted states
    std::vector< PoseEstimate> estiamted_states( K);
    // Set time steps for each state
    for( int i = 0; i < K - 1; i++){
        estiamted_states[i].setTime( 
            meas_control_input[i].time()
        );
    }
    // Set the last estimate time
    estiamted_states[K  - 1].setTime( estiamted_states[K-2].time() + dt_func(K-2));

    // Set the time from the first time element of the control input measurement
    estiamted_states[0].setMean( x_0);
    estiamted_states[0].setCov(cov_P_0);
    
    
    // *********************************************
    //  Run the Kalman filter
    //  Estimate and covariance at the previous step
    VectorPose x_km1;
    CovPose    P_km1;
    //  Estimate and covariance at the current step
    VectorPose x_k;
    CovPose    P_k;
    // Control input and covariance at previous time step
    Eigen::Matrix<double, size_u, 1> u_km1;
    Eigen::Matrix<double, size_u, size_u> Cov_u_km1;
    // DT process model matrices
    MatrixA A_km1;
    MatrixB B_km1;
    MatrixA Q_km1; // Discrete-time process noise covariance
    // Index that keeps track of the gps measurements
    size_t idx_gps = 0;
    for( int i = 1; i < K; i++){                
        // Get the pose estimate and covariance at the previous step
        x_km1 = estiamted_states[i - 1].mean();
        P_km1 = estiamted_states[i - 1].cov();
        u_km1 = meas_control_input[i - 1].mean();
        Cov_u_km1 = meas_control_input[i - 1].cov();

        // Compute DT process model matrices
        double dt;
        if( i < K - 1){
            dt = dt_func(i);
        }// Else, use the same dt from the previous iteration.
        std::tie( A_km1, B_km1, Q_km1) = getDiscreteABQ( sys_A_ct, sys_B_ct, sys_L_ct, Cov_u_km1, dt);

        // Prediction step
        x_k = process_model( x_km1, u_km1, A_km1, B_km1);
        P_k = A_km1 * P_km1 * A_km1. transpose() + Q_km1;
        // Ensure symmetry of the covariance
        P_k = 0.5 * (P_k + P_k.transpose());

        // Check for correction
        if( idx_gps < meas_gps.size() && (meas_gps[idx_gps].time() <= estiamted_states[i].time())){
            // Implement a correction
            auto R_k = meas_gps[idx_gps].cov();
            auto y_k = meas_gps[idx_gps].mean();
            // Compute S_k
            Eigen::Matrix< double, size_y, size_y> S_k = sys_C * P_k * sys_C.transpose() + sys_M * R_k * sys_M.transpose();
            // Ensure symmetry
            S_k = 0.5 * (S_k + S_k.transpose());
            // Compute Kalman gain
            // Eigen::Matrix<double, size_x, size_y> K_k = 
            //     (S_k.ldlt().solve(sys_C * P_k)).transpose();
            Eigen::Matrix<double, size_x, size_y> K_k = 
                (S_k.colPivHouseholderQr().solve(sys_C * P_k)).transpose();
            // Update state estimate (xhat)
            x_k += K_k * (y_k - sys_C * x_k);
            // Update covariance
            P_k = ((Eigen::Matrix<double, size_x, size_x>::Identity()) - K_k * sys_C) * P_k;
            // Ensure symmetry
            P_k = 0.5 * (P_k + P_k.transpose());

            idx_gps++;
        }
        // For now, store estimates
        estiamted_states[i].setMean( x_k);
        estiamted_states[i].setCov(  P_k);
    }

    // Display estimates
    std::cout << "\n\n===============================\nEstimates\n" << std::endl;
    std::cout << "Time\t\tMeas\t\tVar" << std::endl;
    for( auto meas : estiamted_states){
        RV::IO::print( meas);
        std::cout << std::endl;
    }

    

    // ******************************************
    // Exporting data
    std::string file_name_out = "/home/aalbaali/Documents/Data/Data_generator/linear_system/msd_kf_estimates.txt";
    std::cout << "\nExporting estimates to '" << file_name_out << "'" << std::endl;

    // Definer header
    std::vector<std::string> header( size_x * (size_x + 1) + 1);
    // First entry: time
    header[0] = "Time";
    // Second inputs: states
    for(size_t i = 0; i < size_x; i++){
        std::stringstream ss;
        ss << "x_" << i+1;
        ss >> header[1 + i];
    }
    // Third input: covariances
    for(size_t j = 0; j < size_x; j++){
        for(size_t i = 0; i < size_x; i++){
            std::stringstream ss;
            ss << "cov_" << (i+1) << (j+1);
            ss >> header[1 + size_x + size_x * j + i];
        }
    }

    RV::IO::write( estiamted_states, header, file_name_out);
}