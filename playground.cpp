#include <iostream>
#include <vector>

#include <unsupported/Eigen/MatrixFunctions>
#include "Eigen/Dense"

// Include parser
#include "data_parser.h"

namespace Eigen{
    // Vector template
    template< size_t Size, typename T = double>
    using Vector = Eigen::Matrix< T, Size, 1>;

    template< size_t Size>
    using Vectord = Vector< Size, double>;
}

// Class of measurements. Contains i) measurement, ii) covariance, and iii) time of the measurement.
template<size_t Size, typename T = double>
class Measurement{
    public:
        Measurement(){
            _meas = Eigen::Matrix< T, Size, 1>::Zero();
            _cov  = Eigen::Matrix< T, Size, Size>::Zero();
            _t = -1.0;
        }
        

        // // Import measurements from a row matrix of measurements (first element is time, second measurements are measurement values, and then finally, the covariance matrix (n * (n+1)/2 elements)).
        // Measurement( Eigen::Matrix< T, Size * (Size + 1) + 1, 1> meas_row_in){
        //     _t = meas_row_in( 0);

        //     _meas = meas_row_in.segment( 1, Size);
            
        //     Eigen::Matrix< T, Size * Size> cov_vec = meas_row_in.segment( Size + 1, Size * Size);
        //     for( int i; i < Size; i++){
        //         for( int j; j < Size; j++){
        //             _cov( i, j) = cov_vec( Size * i + j, 0);
        //         }
        //     }
        // }

        Measurement( Eigen::Matrix< T, Size, 1> meas_in,
            Eigen::Matrix< T, Size, Size> cov_in = Eigen::Matrix< T, Size, Size>::Identity(), double time_in = -1){
            // Constructor that takes measurement and covariance (with default values).
            _meas = meas_in;
            _cov  = cov_in;
            _t    = time_in;
        }

        // Getters
        Eigen::Matrix< T, Size, 1> meas(){ return _meas;}
        Eigen::Matrix< T, Size, Size> cov(){ return  _cov;}
        double time(){ return _t;}

        // Setters
        void setMeas( Eigen::Matrix< T, Size, 1> meas_in){
            this->_meas = meas_in;
        }

        void setCov( Eigen::Matrix<T, Size, 2> cov_in){
            this->_cov = cov_in;
        }
        void setTime( double time_in){ _t = time_in;}

    private:
        Eigen::Matrix< double, Size, 1> _meas;
        Eigen::Matrix< double, Size, Size> _cov;

        // Time of measurement
        double _t;
};

// Acceleration measurement class (takes a scalar input)
typedef Measurement< 1, double> MeasControlInput;

// // Function that imports a vector of the control input measurements
// std::vector< MeasControlInput> importMeasControlInput( const std::string &file_path){
//     // Import the data
//     auto data_vec = importData( file_path);
//     std::vector< MeasControlInput> vec_meas_u( data_vec.size());
//     for( size_t i = 0; i < data_vec.size(); i++){
//         vec_meas_u[ i] = MeasControlInput( data_vec[ i]);
//     }
// }

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

// Function that extracts the measurement object given the row from the raw data file
template<size_t size_u, typename T>
MeasControlInput getMeasurementObject( std::vector<T> row_of_raw_data){
    double time = row_of_raw_data[ 0];
    Eigen::Vectord< size_u> meas;
    Eigen::Matrix< double, size_u, size_u> cov;
    for( size_t i = 0; i < size_u; i++){
        meas( i) = row_of_raw_data[ i + 1];
        for( size_t j = 0; j < size_u; j++){
            cov( i, j) = row_of_raw_data[ 1 + size_u + i * size_u + j];
        }
    }
    MeasControlInput meas_obj( meas, cov, time);

    return meas_obj;
}
// Control input file name
const std::string file_name_u = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/msd_acc.txt";

int main(){
    // Store the row of data into an approparite object
    //  Get size of the measurement
    const size_t size_u = 1;

    // Import vector data
    auto data_vec = importData( file_name_u);

    // Vector of measurement objects
    std::vector< MeasControlInput> vec_meas_obj( data_vec.size());
    for( int i = 0; i < data_vec.size(); i++){
        vec_meas_obj[ i] = getMeasurementObject< size_u>( data_vec[i]);
    }
    
    // Extract the first row
    auto first_meas = vec_meas_obj[0];
    // Output the fields
    std::cout << "time: " << first_meas.time() << std::endl;
    std::cout << "meas: " << first_meas.meas().transpose() << std::endl;
    std::cout << "cov: " << first_meas.cov() << std::endl;

    
    

    // *********************************************
    // Control inputs
    // auto v_u = importControlInput( K);
    auto v_u = importControlInput( file_name_u);
}