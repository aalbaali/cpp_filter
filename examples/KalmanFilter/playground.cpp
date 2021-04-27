#include <iostream>
#include <vector>
#include <iomanip> // For nice outputs
#include <unsupported/Eigen/MatrixFunctions>
#include "Eigen/Dense"

// Dimension of control input measurements
const size_t size_u = 1;
// Dimension of the exeteroceptive measurement
const size_t size_y = 1;
// Dimension of each pose
const size_t size_x = 2;

typedef Eigen::Matrix< double, size_x, 1>       VectorPose;
typedef Eigen::Matrix< double, size_x, size_x>  CovPose;

typedef Eigen::Matrix< double, size_x, size_x>  MatrixA;
typedef Eigen::Matrix< double, size_x, size_x>  MatrixQ;
typedef Eigen::Matrix< double, size_x, size_u>  MatrixB;

#include <tuple>
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

int main(){
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
        auto sys_L_ct = sys_B_ct;

        Eigen::Matrix<double, 1, 1> Q_ct_km1;
        Q_ct_km1.setIdentity();

         // Declare the discrete-time (DT) system matrices. These are to be computed using a zero-order hold using continous-time (CT) matrices
        Eigen::Matrix2d sys_A;
        Eigen::Vector2d sys_B;
        Eigen::Matrix2d sys_Q;
        // std::tie( sys_A, sys_B, sys_Q) = getDiscreteABQ( sys_A_ct, sys_B_ct, sys_L_ct, Q_ct_km1);

        std::tie( sys_A, sys_B, sys_Q) = getDiscreteABQ( sys_A_ct, sys_B_ct, sys_L_ct, Q_ct_km1, 0.1);
        std::cout << "sys_A:\n" << sys_A << std::endl;
        std::cout << "sys_B:\n" << sys_B << std::endl;
}