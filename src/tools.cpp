#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.empty() || estimations.size() != ground_truth.size()) return rmse;
  //accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    for (int j = 0; j < rmse.size(); j++)
      rmse[j] += pow(estimations[i][j] - ground_truth[i][j], 2);
  }
  // root mean residuals
  for (int i = 0; i < rmse.size(); i++) {
    //calculate the mean
    rmse[i] /= estimations.size();
    //calculate the squared root
    rmse[i] = sqrt(rmse[i]);
  }
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  //recover state parameters
  double px = x_state(0), py = x_state(1), vx = x_state(2), vy = x_state(3);

  //check division by zero
  double den = pow(px, 2) + pow(py, 2);
  if (den == 0) return Hj;
  double den_sqrt = sqrt(den);

  //compute the Jacobian matrix
  Hj << px / den_sqrt, py / den_sqrt, 0, 0,
        -py / den, px / den, 0, 0,
        py * (vx * py - vy * px) / pow(den_sqrt, 3), px * (vy * px - vx * py) / pow(den_sqrt, 3), px / den_sqrt,
        py / den_sqrt;
  return Hj;
}
