/// @brief mathematical computation tools.
/// @file : tools.h
/// @author : s.aparajith@live.com
/// @date : 20/4/2021
/// @details : contains the tool class declaration for jacobian and rmse computation
/// @copyright : none reserved. No liabilities. this source code is free to be distributed and copied. use under own resposibility. MIT-License.

#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
namespace scientific{
class Tools {
 public:
  ///@brief Constructor.
  Tools();

  ///@brief Destructor.
  virtual ~Tools();

  ///@brief A helper method to calculate RMSE.
  ///@param estimations: vector of estimations
  ///@param ground_truth : vector of ground truth
  ///@return vector of 4x1 with RMSE for positions and velocities
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /// @brief A helper method to calculate Jacobians.
  /// @param x_state : state vector
  /// @return Jacobian matrix
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
};
}
#endif  // TOOLS_H_
