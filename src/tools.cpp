#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
using std::vector;
using namespace ranges;

namespace Tools {
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
        ArrayXd zero_vector = ArrayXd::Zero(4);

        auto sum_of_square_diffs =
            accumulate(
                    view::zip(estimations, ground_truth)
                    | view::transform([](auto tuple) { return (tuple.first - tuple.second).array().square(); }),
                    zero_vector);

        return (sum_of_square_diffs / estimations.size()).sqrt();
    }

    MatrixXd CalculateJacobian(const VectorXd &x_state) {
        MatrixXd Hj(3, 4);

        float px = x_state(0), py = x_state(1);
        float vx = x_state(2), vy = x_state(3);

        if (px == 0 && py == 0) {
            throw std::invalid_argument("Both px and py are equal to 0!");
        }

        float dist = sqrt(pow(px, 2) + pow(py, 2));
        Hj <<
            px / dist, py / dist, 0, 0,
            -py / pow(dist, 2), px / pow(dist, 2), 0, 0,
            py * (vx * py - vy * px) / pow(dist, 3), px * (vy * px - vx * py) / pow(dist, 3), px / dist, py / dist;

        return Hj;
    }

    VectorXd Cartesian_to_polar(VectorXd x)
    {
        double px = x(0), py = x(1), ux = x(2), uy = x(3); 

        double measure_x = sqrt(pow(px, 2) + pow(py, 2));
        VectorXd out_x(3);
        out_x << 
            measure_x,
            atan(py / px),
            (px * ux + py * uy) / measure_x;

        return out_x;
    }
}