#include <gtest/gtest.h>

#include "../src/fusion_ekf.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

TEST(FusionEKF, ProcessesLaserAndRadarMeasurements)
{
    VectorXd measurement1_raw(2);
    measurement1_raw << 2.65526, 0.66598;

    MeasurementPackage measurement1;
    measurement1.raw_measurements_ = measurement1_raw;
    measurement1.sensor_type_ = MeasurementPackage::LASER;
    measurement1.timestamp_ = 1477010443050000;

    VectorXd measurement2_raw(3);
    measurement2_raw << 2.99092, 0.217668, 5.19181;

    MeasurementPackage measurement2;
    measurement2.raw_measurements_ = measurement2_raw;
    measurement2.sensor_type_ = MeasurementPackage::RADAR;
    measurement2.timestamp_ = 1477010443100000;

    VectorXd expected_x(4);
    expected_x << 2.9202169516550112, 0.65334342588644378, 5.3019717776028132, 0.19992423339925414;

    FusionEKF fusionEKF;
    fusionEKF.ProcessMeasurement(measurement1);
    fusionEKF.ProcessMeasurement(measurement2);
    
    VectorXd actual_x = fusionEKF.EstimatedLocation();

    ASSERT_EQ(actual_x, expected_x);
}