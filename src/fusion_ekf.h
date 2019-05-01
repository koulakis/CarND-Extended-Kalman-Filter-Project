#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  VectorXd EstimatedLocation();

 private:
  // class constants
  MatrixXd R_laser_;
  MatrixXd R_radar_;
  MatrixXd H_laser_;
  double noise_ax_;
  double noise_ay_;

  // class state
  VectorXd x_;
  MatrixXd P_;

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;
};

#endif // FusionEKF_H_
