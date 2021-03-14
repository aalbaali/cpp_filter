#include <iostream>
#include <vector>

#include <unsupported/Eigen/MatrixFunctions>
#include "Eigen/Dense"

// Include parser
#include "data_parser.h"

// Process model: takes a 'double' control input
Eigen::VectorXd process_model( Eigen::VectorXd x_km1, Eigen::VectorXd u_km1, Eigen::MatrixXd A, Eigen::MatrixXd B){
    // @params[in] x_km1    :   state at k - 1
    // @params[in] u_km1    :   control input of type double
    // @params[in] A        :   state matrix
    // @params[in] B        :   control matrix
    return A * x_km1 + B * u_km1;
}
Eigen::VectorXd process_model( Eigen::VectorXd x_km1, double u_km1, Eigen::MatrixXd A, Eigen::MatrixXd B){
    // @params[in] x_km1    :   state at k - 1
    // @params[in] u_km1    :   control input
    // @params[in] A        :   state matrix
    // @params[in] B        :   control matrix
    return A * x_km1 + B * u_km1;
}

// Function to parse the control inputs
template< typename T = double>
std::vector<T> importControlInput(const std::string &file_path){
    // file_path is a path to the control input .txt file

    // Import the data
    auto data_vec = importData( file_path);
    // From the imported data, import a vector of the control inputs only (ignore the time measurements and variance)
    std::vector< T> vec_u( data_vec.size());
    for( size_t i = 0; i < data_vec.size(); i++){
        // Import the second reading, which is the control input. The first reading is the time value
        vec_u[ i] = data_vec[ i][1];
    }
    return vec_u;
}
// Function to parse the control inputs
template< typename T = double>
std::vector<T> importControlInput(const int K){
    // K : Number of poses

    // This implementation of the function simply generates the data
    std::vector< double> v_u( K - 1);
    // Set all control inputs to 1
    for( int i : v_u){
        v_u[i] = 1.;
    }
    return v_u;
}


// Control input file name
const std::string file_name_u = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/msd_acc.txt";

int main(){

    // Declare the discrete-time (DT) system matrices. These are to be computed using a zero-order hold using continous-time (CT) matrices
    Eigen::Matrix2d sys_A;
    Eigen::Vector2d sys_B;

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
    }

#ifdef DEBUG
    std::cout << "sys_A_dt:\n" << sys_A << std::endl;
    std::cout << "sys_B_dt:\n" << sys_B << std::endl;
#endif

    // Covariances
    //      Process covariance
    Eigen::Matrix2d cov_Q;    
    cov_Q << 1, 0, 0, 1;

    // *********************************************
    // Initial conditions
    Eigen::Vector2d x_0( 0., 0.);
    // Covariance on initial condition
    Eigen::Matrix2d cov_P_0;
    cov_P_0 << 1e-3, 0, 0, 1e-3;

    // *********************************************
    // Simulation inputs
    //  Number of poses
    const unsigned int K = 3;

    // *********************************************
    // Control inputs
    // auto v_u = importControlInput( K);
    auto v_u = importControlInput( file_name_u);
    
    // *********************************************
    //  Estaimted states
    std::vector< Eigen::Vector2d> v_x_est( K);
    //  Estimated covariance
    std::vector< Eigen::Matrix2d> v_cov_P( K);

    // Initialize all estimates to zeros.
    for( int i = 0; i < K; i++){
        v_x_est[ i] << Eigen::Vector2d::Zero();
        v_cov_P[ i] << Eigen::Matrix2d::Zero();
    }

    // Set first pose to the initial condition
    v_x_est[ 0] = x_0;
    v_cov_P[ 0] = cov_P_0;

    // *********************************************
    //  Propagate estiamtes
    for( int i = 1; i < K; i++){
        // Propagate states
        v_x_est[ i] = process_model( v_x_est[ i - 1], v_u[ i - 1], sys_A, sys_B);
        // Propaget covariance
        v_cov_P[ i] = sys_A * v_cov_P[ i - 1] * sys_A. transpose() + cov_Q;
    }

    // Display propagated states
    std::cout << "Propageted states:" << std::endl;
    for( auto x : v_x_est){
        std::cout << x.transpose() << "\n";
    }
    std::cout << std::endl;
    
#ifdef DEBUG
    std::cout << "x_K:\n" << v_x_est[K - 1] << std::endl;
#endif
    // Propagate state
    auto x_1 = process_model( x_0, v_u[0], sys_A, sys_B);

#ifdef DEBUG
    std::cout << "x_1:\t" << x_1 << std::endl;
#endif

}