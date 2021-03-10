#include <iostream>
#include <vector>

#include "Eigen/Dense"

// Process model
Eigen::VectorXd process_model( Eigen::VectorXd x_km1, Eigen::VectorXd u_km1, Eigen::MatrixXd A, Eigen::MatrixXd B){
    // @params[in] x_km1    :   state at k - 1
    // @params[in] u_km1    :   control input
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


int main(){
    // *********************************************
    // System parameters
    // System matrix
    Eigen::Matrix2d sys_A;
    // Control matrix
    Eigen::Vector2d sys_B;
    // System is a mass-spring-damper system with unit mass, spring-constant, and damping.
    sys_A << 0, 1, -1, -1;
    sys_B << 0, 1;

    // Covariances
    //      Process covariance
    Eigen::Matrix2d cov_Q;    
    cov_Q << 1, 0, 0, 1;

#ifdef DEBUG
    std::cout << "sys_A:\n" << sys_A << "\nsys_B:\n" << sys_B << std::endl;
#endif

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
    std::vector< double> v_u( K - 1);
    // Set all control inputs to 1
    for( int i : v_u){
        v_u[i] = 1.;
    }

    // *********************************************
    //  Estaimted states
    std::vector< Eigen::Vector2d> v_x_est( K);
    // Initialize all estimates to zeros.
    for( int i = 0; i < K; i++){
        v_x_est[ i] << Eigen::Vector2d::Zero();
    }
    // Set first pose to the initial condition
    v_x_est[ 0] = x_0;


    // *********************************************
    //  Propagate estiamtes
    for( int i = 1; i < K; i++){
        v_x_est[ i] = process_model( v_x_est[ i - 1], v_u[ i - 1], sys_A, sys_B);
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