#include <gtest/gtest.h>

#include "../src/tools.h"
#include "../src/kalman_filter.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

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
