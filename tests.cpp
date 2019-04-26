//
// Created by mariosk on 26/04/19.
//

#include <gtest/gtest.h>
#include "src/tools.h"
#include <iomanip>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

TEST(RMSE, ComputesCorrectly)
{
    VectorXd meas_pred1(4);
    meas_pred1 << 1,2,3,4;
    VectorXd meas_pred2(4);
    meas_pred2 << 32,4,54,4;
    vector<VectorXd> measurements_pred = {meas_pred1, meas_pred2};

    VectorXd meas_gt1(4);
    meas_gt1 << 43,5,33,3;
    VectorXd meas_gt2(4);
    meas_gt2 << 33,43,4,32;
    vector<VectorXd> measurements_ground_truth = {meas_gt1, meas_gt2};

    VectorXd calculated_rmse = Tools::CalculateRMSE(measurements_pred, measurements_ground_truth);

    VectorXd expected_rmse(4);
    expected_rmse << 29.706901555025897, 27.65863337187866, 41.23105625617661, 19.81161275615895;

    ASSERT_EQ(calculated_rmse, expected_rmse);
}

TEST(Jacobian, ComputedCorrectly)
{
    VectorXd measurement(4);
    measurement << 3, 2, 5, 1;
    MatrixXd expected_jacobian(3, 4);
    expected_jacobian <<
        0.83205032348632812, 0.55470019578933716, 0, 0,
        -0.1538461566312285, 0.23076923494684273, 0, 0,
        0.29868472915498179, -0.44802709373247268, 0.83205032348632812, 0.55470019578933716;

    auto jacobian = Tools::CalculateJacobian(measurement);

    ASSERT_EQ(jacobian, expected_jacobian);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}