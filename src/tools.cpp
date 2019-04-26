#include "tools.h"
#include <iostream>
#include <boost/range/combine.hpp>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  ArrayXd mse(4);
  mse << 0, 0, 0, 0;
  int number_of_samples = estimations.size();

  for(int i = 0; i < number_of_samples; i++) {
    mse += (estimations[i] - ground_truth[i]).array().square();
  }
  
  return (mse / number_of_samples).sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    if(px == 0 && py == 0) {
        throw std::invalid_argument("Both px and py are equal to 0!");
    }

    float dist = sqrt(pow(px, 2) + pow(py, 2));
    Hj <<
        px / dist, py / dist, 0, 0,
        - py / pow(dist, 2), px / pow(dist, 2), 0, 0,
        py * (vx * py - vy * px) / pow(dist, 3), px * (vy * px - vx * py) / pow(dist, 3), px / dist, py / dist;

    return Hj;
}
