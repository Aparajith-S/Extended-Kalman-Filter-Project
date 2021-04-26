#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
namespace fusion{
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


/// @brief Constructor.
FusionEKF::FusionEKF():
ekf_(VectorXd(4),
MatrixXd(4, 4),
MatrixXd(4, 4)) {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  //Measurement matrix - LIDAR
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;
	//Measurement matrix - RADAR
	Hj_ << 1, 1, 0, 0,
		1, 1, 0, 0,
		1, 1, 1, 1; 
  // create a 4D state vector, we don't know yet the values of the x state
	auto x_in = VectorXd(4);
	// state covariance matrix P
	auto P_in = MatrixXd(4, 4);
	P_in << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
	// the initial transition matrix F_
	auto F_in = MatrixXd(4, 4);
	F_in << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
	// set the acceleration noise components
	noise_ax = 9;
	noise_ay = 9; 
  //Q and R inits dont matter much as they will be replaced later.
  ekf_.Init(x_in,
  P_in,
  F_in,
  Hj_,
  MatrixXd(2, 2),
  MatrixXd(4, 4));
}


/// Destructor.
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  //Initialization
  if (!is_initialized_) {
    // Initialize the state ekf_.x_ with the first measurement.
    // Create the covariance matrix.
    //You'll need to convert radar from polar to cartesian coordinates.
    // first measurement
    cout << "EKF: " << endl;
    ekf_.modifyVectorX() = VectorXd(4);
    ekf_.modifyVectorX() << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //  Convert radar from polar to cartesian coordinates 
      //  and initialize state.
			float ro = measurement_pack.raw_measurements_(0);
			float phi = measurement_pack.raw_measurements_(1);
			float roDot = measurement_pack.raw_measurements_(2);
			ekf_.modifyVectorX()(0) = ro     * cos(phi);
			ekf_.modifyVectorX()(1) = ro     * sin(phi);
			ekf_.modifyVectorX()(2) = roDot  * cos(phi);
			ekf_.modifyVectorX()(3) = roDot  * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
			ekf_.modifyVectorX() << measurement_pack.raw_measurements_[0],
				measurement_pack.raw_measurements_[1],
				0,
				0;
    }
		previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  //Prediction
  // Update the state transition matrix F according to the new elapsed time.
  //Time is measured in seconds.
  //Update the process noise covariance matrix.
  //Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  constexpr float kConvert2secs =1000000.0F;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / kConvert2secs;
	previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	// Modify the F matrix so that the time is integrated
  ekf_.modifyFmatrix()(0, 2) = dt;
	ekf_.modifyFmatrix()(1, 3) = dt;

	// set the process covariance matrix Q
	
	ekf_.modifyQmatrix() << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
  
  ekf_.Predict();

   //  Update
   //- Use the sensor type to perform the update step.
   //- Update the state and covariance matrices.
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Tools tools;
		Hj_ = tools.CalculateJacobian(ekf_.getX());
		ekf_.modifyHmatrix() = Hj_;
		ekf_.modifyRmatrix() = R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
		ekf_.modifyHmatrix() = H_laser_;
		ekf_.modifyRmatrix() = R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.getX() << endl;
  cout << "P_ = " << ekf_.getP() << endl;
}
}