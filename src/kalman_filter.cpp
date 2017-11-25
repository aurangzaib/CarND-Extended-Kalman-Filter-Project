#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
/**
 * Predict x and P
 */
void KalmanFilter::Predict() {
  // state vector
  x_ = F_ * x_;
  // state transition matrix
  MatrixXd Ft = F_.transpose();
  // prediction uncertainty covariance matrix
  P_ = F_ * P_ * Ft + Q_;
}
/**
 * update the state by using Kalman Filter equations
 */
void KalmanFilter::Update(const VectorXd &z) {
  // remove velocity components from measurement
  VectorXd y = z - H_ * x_;
  // implement kalman filter
  KF(y);
}
/**
 * update the state by using Extended Kalman Filter equations
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // transform to polar coordinates
  double px = x_(0), 
         py = x_(1),
         vx = x_(2),
         vy = x_(3); 
  double rho = sqrt(pow(px, 2) + pow(py, 2));
  double phi = atan2(py, px);
  const double EPS = 0.0001;
  double rho_dot = (fabs(rho) < EPS) ? 0 : (px * vx + py * vy) / rho; 
  
  // find transformation function
  VectorXd h(3);
  h << rho, phi, rho_dot;

  // predicted measurement
  VectorXd y = z - h;

  // implement kalman filter
  KF(y);
};
/**
 * Kalman filter implementation for laser and ridar
 */
void KalmanFilter::KF(const VectorXd &y) {

  // measurement function
  MatrixXd Ht = H_.transpose();

  // sensor measurement error
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();

  // kalman filter gain
  MatrixXd K = P_ * Ht * Si;

  // new state vector estimate
  x_ = x_ + (K * y);

  // identity matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // new state covariance  matrix
  P_ = (I - K * H_) * P_;
}