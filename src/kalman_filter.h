#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
namespace kalman{
/// @class kalman filter 
/// @brief instantiates a kalman filter object
class KalmanFilter {
 public:
  
  /// @brief Constructor.
  explicit KalmanFilter(Eigen::VectorXd const & x_in,
  Eigen::MatrixXd const & P_in,
  Eigen::MatrixXd const & F_in);

  /// @brief Destructor
  virtual ~KalmanFilter();

   /// @brief Init Initializes Kalman filter
   /// @param x_in Initial state
   /// @param P_in Initial state covariance
   /// @param F_in Transition matrix
   /// @param H_in Measurement matrix
   /// @param R_in Measurement covariance matrix
   /// @param Q_in Process covariance matrix
  void Init(Eigen::VectorXd const &x_in, Eigen::MatrixXd const &P_in, Eigen::MatrixXd const &F_in,
            Eigen::MatrixXd const &H_in, Eigen::MatrixXd const &R_in, Eigen::MatrixXd const &Q_in);

  ///
  ///Prediction Predicts the state and the state covariance
  ///using the process model
  ///@param delta_T Time between k and k+1 in s
  ///
  void Predict();

  ///@brief Updates the state by using standard Kalman Filter equations
  ///@param z The measurement at k+1
  void Update( Eigen::VectorXd const &z);

  /// 
  /// @brief Updates the state by using Extended Kalman Filter equations
  /// @param z The measurement at k+1
  /// 
  void UpdateEKF( Eigen::VectorXd const &z);

  /// @brief expose the member for update
  Eigen::MatrixXd & modifyQmatrix(void);
  /// @brief expose the member for update
  Eigen::MatrixXd & modifyRmatrix(void);
  /// @brief expose the member for update
  Eigen::MatrixXd & modifyFmatrix(void);
  /// @brief expose the member for update
  Eigen::MatrixXd & modifyHmatrix(void);
  /// @brief expose the member for update
  Eigen::VectorXd & modifyVectorX(void);
  Eigen::VectorXd getX(void)const;
  Eigen::MatrixXd getP(void)const;
private:
  /// @brief state vector
  Eigen::VectorXd x_;

  /// @brief state covariance matrix
  Eigen::MatrixXd P_;

  /// @brief state transition matrix
  Eigen::MatrixXd F_;

  /// @brief process covariance matrix
  Eigen::MatrixXd Q_;

  /// @brief measurement matrix
  Eigen::MatrixXd H_;
  
  /// @brief measurement covariance matrix
  Eigen::MatrixXd R_;

  /// @brief Updates the state by Common equations to both EKF and KF
  /// @param y The measurement at k+1
  void KfCommonUpdates(const Eigen::VectorXd &y);
};
}
#endif // KALMAN_FILTER_H_
