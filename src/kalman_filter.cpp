#include "kalman_filter.h"
#include <tuple>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace KalmanFilter {
    std::function<std::tuple<VectorXd, MatrixXd>(const VectorXd&, const MatrixXd&)> Predict(
        const MatrixXd &F, const MatrixXd &Q)
    {
        return [&F, &Q](const VectorXd &x, const MatrixXd &P) -> std::tuple<VectorXd, MatrixXd>
        {
            return std::make_tuple(
                F * x, 
                F * P * F.transpose() + Q
            );
        };
    }

    std::function<std::tuple<VectorXd, MatrixXd>(const VectorXd&, const MatrixXd&, const VectorXd&)> Update(
        const MatrixXd &H, 
        const MatrixXd &R)
    {
        return [&H, &R](const VectorXd& x, const MatrixXd& P, const VectorXd& z) -> std::tuple<VectorXd, MatrixXd> {
            MatrixXd I = MatrixXd::Identity(P.rows(), P.cols());

            VectorXd y = z - H * x;
            MatrixXd S = H * P * H.transpose() + R;
            MatrixXd K = P * H.transpose() * S.inverse();

            MatrixXd new_x = x + K * y;
            MatrixXd new_P = (I - K * H) * P;
        
            return std::make_tuple(new_x, new_P);
        };
    }

    std::function<std::tuple<VectorXd, MatrixXd>(const VectorXd&, const MatrixXd&, const VectorXd&)> UpdateEKF(
        const MatrixXd &R)
    {
        return [&R](const VectorXd& x, const MatrixXd& P, const VectorXd& z) -> std::tuple<VectorXd, MatrixXd> {
            MatrixXd H = Tools::CalculateJacobian(x);

            MatrixXd I = MatrixXd::Identity(P.rows(), P.cols());

            VectorXd y = z - Tools::Cartesian_to_polar(x);

            // Pick the corresponding angle in the interval [-pi, pi]
            y[1] = Tools::NormalizeAngle(y[1]);

            MatrixXd S = H * P * H.transpose() + R;
            MatrixXd K = P * H.transpose() * S.inverse();

            MatrixXd new_x = x + K * y;
            MatrixXd new_P = (I - K * H) * P;
        
            return std::make_tuple(new_x, new_P);
        };
    }
}