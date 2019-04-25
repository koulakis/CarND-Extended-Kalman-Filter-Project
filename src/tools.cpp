#include "tools.h"
#include <iostream>
#include <boost/range/combine.hpp>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  // auto x =  boost::combine(estimations, ground_truth);
  VectorXd y(4);
  y << 2,3,4,5;
  return y;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
