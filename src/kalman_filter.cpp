#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // prediction state vector
  x_ = F_ * x_;
  // state transition matrix
  MatrixXd Ft = F_.transpose();
  // prediction uncertainty covariance matrix
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // sensor measurement
  VectorXd y = z - (H_ * x_);
  // measurement function
  MatrixXd Ht = H_.transpose();
  // sensor measurement error
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  // kalman filter gain
  MatrixXd K = P_ * Ht * Si;
  // new estimate
  x_ = x_ + (K * y);
  // identity matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // new prediction covariance  matrix
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // find the jacobian matrix
  Tools tools;
  H_ = tools.CalculateJacobian(x_);
  // sensor measurement
  VectorXd y = z - (H_ * x_);
  // measurement function
  MatrixXd Ht = H_.transpose();
  // sensor measurement error
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  // kalman filter gain
  MatrixXd K = P_ * Ht * Si;
  // new estimate
  x_ = x_ + (K * y);
  // identity matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // new prediction covariance  matrix
  P_ = (I - K * H_) * P_;
};
