/// @brief mathematical computation tools.
/// @file : tools.cpp
/// @author : s.aparajith@live.com
/// @date : 20/4/2021
/// @details : contains the tool class definition for jacobian and rmse computation
/// @copyright : none reserved. No liabilities. this source code is free to be distributed and copied. use under own resposibility. MIT-License.
#include "tools.h"
#include "types.h"
#include <iostream>
namespace scientific
{
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
 VectorXd rmse(4);
  rmse << 0.F,0.F,0.F,0.F;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if((estimations.size() == ground_truth.size())&& estimations.size()!=0)
  {
for (type::uint32 i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();
    
  }
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
 MatrixXd Hj(3,4);
  // recover state parameters
  type::float32 px = x_state(0);
  type::float32 py = x_state(1);
  type::float32 vx = x_state(2);
  type::float32 vy = x_state(3);

  // TODO: YOUR CODE HERE 
  type::float32 sq_term =(px*px + py*py);
  type::float32 sqrt_term = sqrtf(sq_term);
  type::float32 cube_sqrt_term = sqrt_term*sqrt_term*sqrt_term;
  // check division by zero
  if (fabs(px-0.F) < std::numeric_limits<type::float32>::epsilon() &&
  fabs(py-0.F) < std::numeric_limits<type::float32>::epsilon())
  {
      std::cout<<"div by 0 error! Both x and y are 0";
  }
  else{
  // compute the Jacobian matrix
  Hj<< px/sqrt_term, py/sqrt_term,0,0,
       -py/sq_term , px/sq_term  ,0,0,
       (py*(vx*py-vy*px))/cube_sqrt_term,(px*(vy*px-vx*py))/cube_sqrt_term, px/sqrt_term,py/sqrt_term;
  }
  return Hj;
}
}