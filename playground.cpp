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

        Measurement( Eigen::Matrix< T, Size, 1> meas_in,
            Eigen::Matrix< T, Size, Size> cov_in = Eigen::Matrix< T, Size, Size>::Identity(), double time_in = -1){
            // Constructor that takes measurement and covariance (with default values).
            _meas = meas_in;
            _cov  = cov_in;
            _t    = time_in;
        }

        template<typename VectorT>
        Measurement( std::vector<VectorT> row_of_raw_data){
            double time = row_of_raw_data[ 0];
            Eigen::Vectord< Size> meas;
            Eigen::Matrix< double, Size, Size> cov;
            for( size_t i = 0; i < Size; i++){
                meas( i) = row_of_raw_data[ i + 1];
                for( size_t j = 0; j < Size; j++){
                    cov( i, j) = row_of_raw_data[ 1 + Size + i * Size + j];
                }
            }
            // Store objects
            _t    = time;
            _meas = meas;
            // Ensure symmetry of the covariance matrix
            _cov  =  (1/2) * ( cov + cov.transpose());
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

const size_t size_u = 1;
// Acceleration measurement class (takes a scalar input)
typedef Measurement< size_u, double> MeasControlInput;


// Control input file name
const std::string file_name_u = "/home/aalbaali/Documents/Code_base/Examples/Data_generator/linear_system/data/msd_acc.txt";

int main(){

    // Import vector data
    auto data_vec = importData( file_name_u);

    // Vector of measurement objects
    std::vector< MeasControlInput> vec_meas_obj( data_vec.size());
    for( int i = 0; i < data_vec.size(); i++){
        // vec_meas_obj[ i] = getMeasurementObject< MeasControlInput, size_u>( data_vec[i]);
        vec_meas_obj[ i] = MeasControlInput( data_vec[i]);
    }
    
    // Extract the first row
    auto first_meas = vec_meas_obj[10];
    // Output the fields
    std::cout << "time: " << first_meas.time() << std::endl;
    std::cout << "meas: " << first_meas.meas().transpose() << std::endl;
    std::cout << "cov: " << first_meas.cov() << std::endl;
}