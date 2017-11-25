#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor
 */
FusionEKF::FusionEKF() {

  // set initialization false
  is_initialized_ = false;

  // set which sensors to use
  use_radar_ = true;
  use_laser_ = true;

  // initialize timestamp
  previous_timestamp_ = 0;

  // initializing noise and measurement matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  // measurement function - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // state transition matrix            
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  STEP 1 -- Initialization
   ****************************************************************************/

  if (!is_initialized_) {

    // create the covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,    // high certainty in px
               0, 1, 0, 0,    // high certainty in py
               0, 0, 1000, 0, // high uncertainty in vx
               0, 0, 0, 1000; // high uncertainty in vy

    // first measurement
    ekf_.x_ = VectorXd(4);
    
    // radar measurements
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // get sensor measurements
      double rho = measurement_pack.raw_measurements_[0],
             phi = measurement_pack.raw_measurements_[1],
             rho_dot = measurement_pack.raw_measurements_[2];
      // Convert radar from polar to cartesian coordinates
      // initialize state
      ekf_.x_ << rho     * cos(phi),  // px
                 rho     * sin(phi),  // py
                 rho_dot * cos(phi),  // vx
                 rho_dot * sin(phi);  // vy
    }
    // laser measurements
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      // get sensor measurements
      // lidar doesn't provide velocity information
      const float EPS = 0.0001;
      auto px = measurement_pack.raw_measurements_[0],
           py = measurement_pack.raw_measurements_[1];
      // handling zero value case
      if (px < EPS && py < EPS) {
        px = EPS;
        py = EPS;
      }
      // Initialize state
      ekf_.x_ << px,   // px
                 py,   // py
                 1.0,  // vx
                 1.0;  // vy
    }

    // update timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  STEP 2 -- Prediction
   ****************************************************************************/

  /***************************
   *  Define noise components
   ***************************/
  const double noise_ax = 9.0, noise_ay = 9.0;

  // compute the time elapsed between the current and previous measurements
  // difference of current and previous timestamps
  // dt - expressed in seconds
  const double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  /***************************
   *  State transition matrix F
   ***************************/

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  /***************************
   * Process covariance matrix Q
   ***************************/

  // 2nd, 3rd and 4th orders of dt
  double dt2 = pow(dt, 2), dt3 = pow(dt, 3), dt4 = pow(dt, 4);
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4 * noise_ax / 4, 0, dt3 * noise_ax / 2, 0,
             0, dt4 * noise_ay / 4, 0, dt3 * noise_ay / 2,
             dt3 * noise_ax / 2, 0, dt2 * noise_ax, 0,
             0, dt3 * noise_ay / 2, 0, dt2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  STEP 3 -- Update
   ****************************************************************************/

  /***************************
   * Update x and P based on sensor
   ***************************/

  // Radar updates
  // set H and R for radar
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  // Laser updates
  // set H and R for laser
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}