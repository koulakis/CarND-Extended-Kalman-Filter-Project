#include "kalman_filter.h"
#include <tuple>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

namespace Kalman {
    std::function<std::tuple<VectorXd, MatrixXd>(VectorXd, MatrixXd)> Predict(const MatrixXd &F, const MatrixXd &Q) {
        return [&F, &Q](VectorXd x, MatrixXd P) -> std::tuple<VectorXd, MatrixXd> {
            return std::make_tuple(F * x, F * P * F.transpose() + Q);
        };
    }
}

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
    auto pair = Kalman::Predict(F_, Q_)(x_, P_);
    x_ = std::get<0>(pair), P_ = std::get<1>(pair);
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
