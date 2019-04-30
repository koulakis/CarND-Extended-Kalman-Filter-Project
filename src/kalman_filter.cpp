#include "kalman_filter.h"
#include <tuple>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

namespace Kalman {
    std::function<std::tuple<VectorXd, MatrixXd>(VectorXd, MatrixXd)> Predict(const MatrixXd &F, const MatrixXd &Q)
    {
        return [&F, &Q](VectorXd x, MatrixXd P) -> std::tuple<VectorXd, MatrixXd>
        {
            return std::make_tuple(F * x, F * P * F.transpose() + Q);
        };
    }

    std::function<std::tuple<VectorXd, MatrixXd>(VectorXd, MatrixXd, VectorXd)> Update(
        const MatrixXd &H, 
        const MatrixXd &R, 
        const std::function<VectorXd(VectorXd)> &transform_x)
    {
        return [&H, &R, &transform_x](VectorXd x, MatrixXd P, VectorXd z) -> std::tuple<VectorXd, MatrixXd> {
            MatrixXd I = MatrixXd::Identity(P.rows(), P.cols());

            VectorXd y = z - transform_x(x);
            MatrixXd S = H * P * H.transpose() + R;
            MatrixXd K = P * H.transpose() * S.inverse();

            MatrixXd new_x = x + K * y;
            MatrixXd new_P = (I - K * H) * P;
        
            return std::make_tuple(new_x, new_P);
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
    auto transform_x = [this](VectorXd x){return H_ * x;};
    auto pair = Kalman::Update(H_, R_, transform_x)(x_, P_, z);
    x_ = std::get<0>(pair), P_ = std::get<1>(pair);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    MatrixXd H_Jacobian = Tools::CalculateJacobian(x_);

    auto pair = Kalman::Update(H_Jacobian, R_, Tools::Cartesian_to_polar)(x_, P_, z);
    x_ = std::get<0>(pair), P_ = std::get<1>(pair);
}
