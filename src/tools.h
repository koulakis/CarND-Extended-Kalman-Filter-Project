#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include <range/v3/all.hpp>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace Tools {
    /**
    * A helper function to calculate RMSE.
    */
    VectorXd CalculateRMSE(
            const std::vector<VectorXd> &estimations,
            const std::vector<VectorXd> &ground_truth);

    /**
    * A helper function to calculate Jacobians.
    */
    MatrixXd CalculateJacobian(const VectorXd &x_state);
    VectorXd Cartesian_to_polar(VectorXd x);
}

#endif  // TOOLS_H_
