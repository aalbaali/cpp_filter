namespace Eigen{
    // Vector template
    template< size_t Size, typename T = double>
    using Vector = Eigen::Matrix< T, Size, 1>;

    template< size_t Size>
    using Vectord = Vector< Size, double>;
}

// Class of measurements. Contains i) measurement, ii) covariance, and iii) time of the measurement.
// Change the name of the class to RandomVariable (it just includes the mean, covariance, and time step)
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
            _cov  =  0.5 * (cov + cov.transpose());
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


template<typename T>
std::vector< T> ImportMeasurementsObjectVector(const std::string &file_name){
    // Import raw vector data
    auto raw_data = importData( file_name);
    // Vector of measurement objects
    std::vector< T> vector_of_measuremen_objects( raw_data.size());
    for( int i = 0; i < raw_data.size(); i++){
        // meas_control_input[ i] = getMeasurementObject< MeasControlInput, size_u>( raw_data[i]);
        vector_of_measuremen_objects[ i] = T( raw_data[i]);
    }

    return vector_of_measuremen_objects;
}

// Function that displays the random variable
template<typename T>
void displayRV(T rv){
    std::cout << std::setw(1) << rv.time() << "\t\t";
    std::cout << std::setw(5) << rv.meas().transpose() << "\t\t";
    // Vectorize covariance matrix
    Eigen::Map<Eigen::RowVectorXd> cov_vec( rv.cov().data(), rv.cov().size());
    std::cout << std::setw(5) << cov_vec;
}
