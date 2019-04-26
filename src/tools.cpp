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
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
