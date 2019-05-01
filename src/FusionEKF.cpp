#include "FusionEKF.h"
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
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ <<
    0.0225, 0,
    0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ <<
    0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  x_ = VectorXd(4);
  F_ = MatrixXd(4, 4);
  P_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

VectorXd FusionEKF::getX(){ return x_; };

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    
    x_ << 1, 1, 1, 1;

    F_ << 
      1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

    
    P_ << 
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      VectorXd x_location = Tools::PolarToCartesianLocation(measurement_pack.raw_measurements_);
      x_ << x_location(0), x_location(1), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  F_ <<
    1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1;

  double noise_ax = 9, noise_ay = 9;

  Q_ << 
    pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
    0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
    pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
    0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

  auto prediction = KalmanFilter::Predict(F_, Q_)(x_, P_);
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
