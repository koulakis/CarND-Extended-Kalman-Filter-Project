#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace KalmanFilter
{
    std::function<std::tuple<VectorXd, MatrixXd>(const VectorXd&, const MatrixXd&)> Predict(
      const MatrixXd &F, 
      const MatrixXd &Q);
    std::function<std::tuple<VectorXd, MatrixXd>(const VectorXd&, const MatrixXd&, const VectorXd&)> Update(
      const MatrixXd &H, 
      const MatrixXd &R);
    std::function<std::tuple<VectorXd, MatrixXd>(const VectorXd&, const MatrixXd&, const VectorXd&)> UpdateEKF(
      const MatrixXd &R);
}

#endif // KALMAN_FILTER_H_