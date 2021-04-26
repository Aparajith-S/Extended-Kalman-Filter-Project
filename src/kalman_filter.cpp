#include "kalman_filter.h"
#include "tools.h"
#include <limits.h>
namespace kalman{
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  //Checking the value is not zero
  if(px == 0. && py == 0.)
    return;
  
  float rho = sqrt(px*px + py*py);
  float theta = atan2(py, px);
  
  //Checking the value is not zero
  if (rho < std::numeric_limits<float>::epsilon()) 
  {
    rho = std::numeric_limits<float>::epsilon() ;
  } 
  float rho_dot = (px*vx + py*vy) / rho;
  
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
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
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
Eigen::VectorXd KalmanFilter::getX(void)const
{
  return x_;
}
Eigen::MatrixXd KalmanFilter::getP(void)const
{
  return P_;
}
}
