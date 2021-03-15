#include <iostream>
#include <vector>
#include <iomanip> // For nice outputs (setw)

#include <unsupported/Eigen/MatrixFunctions>
#include "Eigen/Dense"

// Include parser
#include "data_parser.h"
#include "Measurement.h"

Eigen::VectorXd process_model( Eigen::VectorXd x_km1, Eigen::VectorXd u_km1, Eigen::MatrixXd A, Eigen::MatrixXd B){
    return A * x_km1 + B * u_km1;
}

// Dimension of control input measurements
const size_t size_u = 1;
// Dimension of the exeteroceptive measurement
const size_t size_y = 1;
// Dimension of each pose
const size_t size_x = 2;

typedef Eigen::Matrix< double, size_x, 1>       VectorPose;
typedef Eigen::Matrix< double, size_x, size_x>  CovPose;

// Acceleration measurement class
typedef Measurement< size_u> MeasControlInput;
typedef Measurement< size_y> MeasGPS;
typedef Measurement< size_x> PoseEstimate;
// Control input file name
const std::string file_name_u = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/msd_acc.txt";

const std::string file_name_gps = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/msd_pos.txt";


int main(){

    // Declare the discrete-time (DT) system matrices. These are to be computed using a zero-order hold using continous-time (CT) matrices
    Eigen::Matrix2d sys_A;
    Eigen::Vector2d sys_B;
    Eigen::Vector2d sys_L;

    // GPS sensor
    Eigen::Matrix<double, size_y, size_x> sys_C;
    sys_C << 1, 0;
    Eigen::Matrix<double, size_y, size_y> sys_M;
    sys_M.setIdentity();

    {
        // The parameters defined in this scope are only needed to compute the DT matrices. Therefore, they'll be deleted once they leave the scope.
        // *********************************************
        // Cotinuous-time (CT) system parameters
        // Continuous-time system matrix
        Eigen::Matrix2d sys_A_ct;
        // Continuous-time control matrix
        Eigen::Vector2d sys_B_ct;
        // System is a mass-spring-damper system with unit mass, spring-constant, and damping.
        sys_A_ct << 0, 1, -1, -1;
        sys_B_ct << 0, 1;

        // *********************************************
        //  Compute discrete-time (DT) system parameters
        // The computed matrix is [A, B; 0, 1]
        Eigen::Matrix<double, 3, 3> mat_AB_I;
        // Set to identity
        mat_AB_I.setZero();
        // Set the two blocks
        mat_AB_I.block<2, 2>(0, 0) = sys_A_ct;
        mat_AB_I.block<2, 1>(0, 2) = sys_B_ct;

        // Compute exponential matrix (includes the DT matrices)
        mat_AB_I = mat_AB_I.exp();    

    #ifdef DEBUG
        std::cout << "mat_AB_I:\n" << mat_AB_I << std::endl;
    #endif

        // Extract the associated blocks
        sys_A = mat_AB_I.block< 2, 2>(0, 0);
        sys_B = mat_AB_I.block< 2, 1>(0, 2);
        sys_L = sys_B;
    }

#ifdef DEBUG
    std::cout << "sys_A_dt:\n" << sys_A << std::endl;
    std::cout << "sys_B_dt:\n" << sys_B << std::endl;
#endif

    // *********************************************
    // Initial conditions
    VectorPose x_0( 0., 0.);
    // Covariance on initial condition
    CovPose cov_P_0;
    cov_P_0 << 1e-3, 0, 0, 1e-3;


    // ************************************************
    // Import control input
    std::vector< MeasControlInput> meas_control_input = ImportMeasurementsObjectVector<MeasControlInput>( file_name_u);
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
    std::vector< MeasGPS> meas_gps = ImportMeasurementsObjectVector<MeasGPS>( file_name_gps);

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
    estiamted_states[0].setMeas( x_0);
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
    Eigen::Matrix<double, size_u, size_u> Q_km1;
    // Index that keeps track of the gps measurements
    size_t idx_gps = 0;
    for( int i = 1; i < K; i++){
        // Get the pose estimate and covariance at the previous step
        x_km1 = estiamted_states[i - 1].meas();
        P_km1 = estiamted_states[i - 1].cov();
        u_km1 = meas_control_input[i - 1].meas();
        Q_km1 = meas_control_input[i - 1].cov();
        // Prediction step
        x_k = process_model( x_km1, u_km1, sys_A, sys_B);
        P_k = sys_A * P_km1 * sys_A. transpose() + sys_L * Q_km1 * sys_L. transpose();
        
        // Ensure symmetry of the covariance
        P_k = 0.5 * (P_k + P_k.transpose());

        // Check for correction
        if( meas_gps[idx_gps].time() <= estiamted_states[i].time()){
            // Implement a correction
            auto R_k = meas_gps[idx_gps].cov();
            auto y_k = meas_gps[idx_gps].meas();
            // Compute S_k
            Eigen::Matrix< double, size_y, size_y> S_k = sys_C * P_k * sys_C.transpose() + sys_M * R_k * sys_M.transpose();
            // Ensure symmetry
            S_k = 0.5 * (S_k + S_k.transpose());
            // Compute Kalman gain
            Eigen::Matrix<double, size_x, size_y> K_k = 
                (S_k.ldlt().solve(sys_C * P_k)).transpose();
            // Update state estimate (xhat)
            x_k += K_k * (y_k - sys_C * x_k);
            // Update covariance
            P_k = ((Eigen::Matrix<double, size_x, size_x>::Identity()) - K_k * sys_C) * P_k;
            // Ensure symmetry
            P_k = 0.5 * (P_k + P_k.transpose());

            idx_gps++;
        }
        // For now, store estimates
        estiamted_states[i].setMeas( x_k);
        estiamted_states[i].setCov(  P_k);
    }

    // Display estimates
    std::cout << "\n\n===============================\nEstimates\n" << std::endl;
    std::cout << "Time\t\tMeas\t\tVar" << std::endl;
    for( auto meas : estiamted_states){
        displayRV( meas);
        std::cout << std::endl;
    }

// #ifdef DEBUG
//     // Display propagated states
//     std::cout << "Propageted states:" << std::endl;
//     for( auto x : estiamted_states){
//         std::cout << x.meas().transpose() << "\n";
//     }
//     std::cout << std::endl;
// #endif
}