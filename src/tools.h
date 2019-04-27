#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include <range/v3/all.hpp>

namespace Tools {
    /**
    * A helper function to calculate RMSE.
    */
    Eigen::VectorXd CalculateRMSE(
            const std::vector<Eigen::VectorXd> &estimations,
            const std::vector<Eigen::VectorXd> &ground_truth);

    /**
    * A helper function to calculate Jacobians.
    */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);
}

#endif  // TOOLS_H_
