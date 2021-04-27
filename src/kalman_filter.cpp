/// @brief kalman filter class definition
/// @file : kalman_filter.cpp
/// @author : s.aparajith@live.com
/// @date : 20/4/2021
/// @details : contains the class definition for extended kalman filter and kalman filter. 
/// @copyright : none reserved. No liabilities. this source code is free to be distributed and copied. use under own resposibility. MIT-License.

#include "kalman_filter.h"
#include "tools.h"
#include "types.h"
#include <limits.h>
namespace kalman{
using Eigen::MatrixXd;
using Eigen::VectorXd;
/// @note
/// The Eigen library does not initialize 
/// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter(VectorXd const & x_in,
  MatrixXd const & P_in,
  MatrixXd const & F_in):
  x_(x_in),
  P_(P_in),
  F_(F_in)
  {};
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd const &x_in, MatrixXd const &P_in, MatrixXd const &F_in,
                        MatrixXd const &H_in, MatrixXd const &R_in, MatrixXd const &Q_in) 
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  KfCommonUpdates(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
type::float32 px = x_[0];
  type::float32 py = x_[1];
  type::float32 vx = x_[2];
  type::float32 vy = x_[3];

  //Checking the value is not zero
  if(px == 0.F && py == 0.F)
    return;
  
  type::float32 rho = sqrtf(px*px + py*py);
  type::float32 theta = atan2f(py, px);
  
  //Checking the value is not zero
  if (rho < std::numeric_limits<type::float32>::epsilon()) 
  {
    rho = std::numeric_limits<type::float32>::epsilon() ;
  } 
  type::float32 rho_dot = (px*vx + py*vy) / rho;
  
  //Finding h(x)
  VectorXd h = VectorXd(3); // h(x_)
  h << rho, theta, rho_dot;
  VectorXd y = z-h;

  //Normalize the angle between -pi to pi
  while (y[1] < -M_PI)
    y[1] += 2 * M_PI;
  while (y[1] > M_PI)
    y[1] -= 2 * M_PI;
  
  //Common measurement update
  KfCommonUpdates(y) ;
}

void KalmanFilter::KfCommonUpdates(const Eigen::VectorXd &y)
{ 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimates
  x_ = x_ + (K * y);
  type::uint32 x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

Eigen::MatrixXd & KalmanFilter::modifyQmatrix(void)
{
  return this->Q_;
}
Eigen::MatrixXd & KalmanFilter::modifyRmatrix(void)
{
  return this->R_;
}
Eigen::MatrixXd & KalmanFilter::modifyFmatrix(void)
{
  return this->F_;
}
Eigen::MatrixXd & KalmanFilter::modifyHmatrix(void)
{
  return this->H_;
}
Eigen::VectorXd & KalmanFilter::modifyVectorX(void)
{
  return this->x_;
}
Eigen::VectorXd const & KalmanFilter::getX(void)const
{
  return x_;
}
Eigen::MatrixXd const & KalmanFilter::getP(void)const
{
  return P_;
}
}
