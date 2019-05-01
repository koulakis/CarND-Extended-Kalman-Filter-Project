//
// Created by mariosk on 26/04/19.
//

#include <gtest/gtest.h>
#include <iomanip>
#include <iostream>

#include "src/tools.h"
#include "src/kalman_filter.h"

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

TEST(KalmanFilterPredict, Predicts)
{
    MatrixXd F(2, 2);
    F <<
        1, 1,
        0, 1;

    MatrixXd Q(2, 2);
    Q <<
        2.1, 1.1,
        0.3, 1.4;

    VectorXd x_old(2);
    x_old << 1, 2;

    MatrixXd P_old(2, 2);
    P_old <<
        0.3, 1.4,
        4., 2;

    VectorXd expected_x(2);
    expected_x << 3, 2;

    MatrixXd expected_P(2, 2);
    expected_P <<
        9.7999999999999989, 4.5,
        6.2999999999999998,  3.3999999999999999;


    auto pair = KalmanFilter::Predict(F, Q)(x_old, P_old);
    auto x = std::get<0>(pair);
    auto P = std::get<1>(pair);

    ASSERT_EQ(x, expected_x);
    ASSERT_EQ(P, expected_P);
}

TEST(KalmanFilterUpdate, Updates)
{
    MatrixXd H(1, 2);
    H << 1, 0;
    
    MatrixXd R(1, 1);
    R << 1;

    VectorXd x_old(2);
    x_old << 0, 0;

    MatrixXd P_old(2, 2);
    P_old << 1000, 0, 0, 1000;

    VectorXd expected_x(2);
    expected_x << 1.9980019980019981, 0;

    MatrixXd expected_P(2, 2);
    expected_P <<
        0.99900099900096517, 0,
        0, 1000;

    VectorXd x(2);
    x = x_old;
    MatrixXd P(2, 2);
    P = P_old;

    VectorXd measurement(1);
    measurement << 2;

    auto pair = KalmanFilter::Update(H, R)(x, P, measurement);
    x = std::get<0>(pair);
    P = std::get<1>(pair);

    ASSERT_EQ(x, expected_x);
    ASSERT_EQ(P, expected_P);
}

TEST(CartesianToPolar, TransformsCorrectly)
{
    VectorXd cartesian(4);
    cartesian << 2, 3, 4, -1.2;
    VectorXd expected_polar(3);
    expected_polar << 3.605551275463989, 0.982793723247329, 1.2203404316955042;

    VectorXd polar = Tools::Cartesian_to_polar(cartesian);

    ASSERT_EQ(polar, expected_polar);
}

TEST(ExtendedKalmanFilterUpdate, Updates)
{
    MatrixXd H(3, 4);
    H << 
        1, 0, 3, 4, 
        5, 4, 3, 2, 
        1, 2, 1, 1;
    
    MatrixXd R(3, 3);
    R << 
        2, 3, 4,
        2, 1, 1,
        2, 5, 3;

    VectorXd x_old(4);
    x_old << 1, 2.3, 3, 2;

    MatrixXd P_old(4, 4);
    P_old << 
        1000, 0, 0, 0,
        0, 1000, 0, 0, 
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    VectorXd expected_x(4);
    expected_x <<
        -6.0696528555076945,
        4.7051670519652893,
        5.0375079628568189,
        6.6862682536542115;

    MatrixXd expected_P(4, 4);
    expected_P <<
        0.95200020906638372, -4.5197403090809809, -1.0979698608941266, -2.5253306472298953,
        -6.8827480367767802, 7.0401678105003196,  4.6557641904209408,  10.708257498772277,
        -5.3004940672112255, 4.7847721133135597,  843.88291460750565,  -359.06929173522065,
        -12.191136196114005, 11.004975717568311,  -359.06929173522059, 174.14063974428052;

    VectorXd x(4);
    x = x_old;
    MatrixXd P(4, 3);
    P = P_old;

    VectorXd measurement(3);
    measurement << 2, 3, 2.5;

    auto transform_x = [H](VectorXd x_) {return H * x_;};
    MatrixXd H_Jacobian = Tools::CalculateJacobian(x_old);

    auto pair = KalmanFilter::Update(H_Jacobian, R)(x_old, P_old, measurement);
    x = std::get<0>(pair), P = std::get<1>(pair);

    ASSERT_EQ(x, expected_x);
    ASSERT_EQ(P, expected_P);
}

TEST(NormalizeAngle, NormalizesPositiveAngle)
{
    ASSERT_EQ(Tools::NormalizeAngle(4343.5), 1.818952738905864);
}

TEST(NormalizeAngle, NormalizesNegativeAngle)
{
    ASSERT_EQ(Tools::NormalizeAngle(-743.2), -1.7841337528088843);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}