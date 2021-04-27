/// @brief fusioning algorithm class.
/// @file : FusionEKF.h
/// @author : s.aparajith@live.com
/// @date : 20/4/2021
/// @details : contains the class definition for a sensor fusion algorithm using EKF 
/// @copyright : none reserved. No liabilities. this source code is free to be distributed and copied. use under own resposibility. MIT-License.

#ifndef FusionEKF_H_
#define FusionEKF_H_
#include "types.h"
#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"
namespace fusion{
class FusionEKF {
 public:
  ///Constructor.
  FusionEKF(void);
  ///Destructor.
  virtual ~FusionEKF(void);

  ///@brief Run the whole flow of the Kalman Filter from here.
  ///@param measurement_pack : gets raw measurements with info whether they are Lidar or radar
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  
  ///@brief gets vector x for display purposes on the sim.
  Eigen::VectorXd const & getVectorX(void)const;

 private:
  ///@brief Kalman Filter update and prediction math lives in here.
  kalman::KalmanFilter ekf_;

  /// @brief check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  type::uint64 previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  scientific::Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  type::float32 noise_ax;
	type::float32 noise_ay; 
  type::float32 dt_stored;
};
}
#endif // FusionEKF_H_
