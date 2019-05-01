#include "fusion_ekf.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() 
{
  /** 
   * initializing constant matrices
   * here it is assumed that the laser & radar observation noise and the laser model matrix are constant 
   * and we hardcode them in the FusionEKF class
   */
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  H_laser_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  R_laser_ <<
    0.0225, 0,
    0, 0.0225; 
  
  R_radar_ <<
    0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
  
  noise_ax_ = 9;
  noise_ay_ = 9;

  // initialize the class state 
  x_ = VectorXd(4);
  P_ = MatrixXd(4, 4);

  is_initialized_ = false;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

// Accessor of the 
VectorXd FusionEKF::EstimatedLocation(){ return x_; };

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    
    // initialize x
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      VectorXd x_location = Tools::PolarToCartesianLocation(measurement_pack.raw_measurements_);
      x_ << x_location(0), x_location(1), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // initialize P
    P_ << 
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

    // initialize timestamp and set initialization to true
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;

    return;
  }

  /**
   * Prediction
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  MatrixXd F = MatrixXd(4, 4);
  F <<
    1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1;

  MatrixXd Q = MatrixXd(4, 4);
  Q << 
    pow(dt, 4) / 4 * noise_ax_, 0, pow(dt, 3) / 2 * noise_ax_, 0,
    0, pow(dt, 4) / 4 * noise_ay_, 0, pow(dt, 3) / 2 * noise_ay_,
    pow(dt, 3) / 2 * noise_ax_, 0, pow(dt, 2) * noise_ax_, 0,
    0, pow(dt, 3) / 2 * noise_ay_, 0, pow(dt, 2) * noise_ay_;

  auto prediction = KalmanFilter::Predict(F, Q)(x_, P_);
  x_ = std::get<0>(prediction), P_ = std::get<1>(prediction);

  /**
   * Update
   */
  auto update = 
    measurement_pack.sensor_type_ == MeasurementPackage::RADAR 
    ? KalmanFilter::UpdateEKF(R_radar_)(x_, P_, measurement_pack.raw_measurements_)
    : KalmanFilter::Update(H_laser_, R_laser_)(x_, P_, measurement_pack.raw_measurements_);

  x_ = std::get<0>(update), P_ = std::get<1>(update);

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}
