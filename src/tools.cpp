#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse.setZero();

  // Check input validity
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Invalid input for calculate RMSE" << endl;
    return rmse;
  }

  // Compute RMSE
  for (int i = 0; i < estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  Hj.setZero();

  // Read state parameters
  double px = x_state[0];
  double py = x_state[1];
  double vx = x_state[2];
  double vy = x_state[3];

  if (px == 0 && py == 0) {
    return Hj;
  }

  //Pre-compute parameters for reuse
  double c1 = px*px+py*py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);

  //Check division by zero
  if(c1 < 0.0000001){
    return Hj;
  }

  //Compute the Jacobian matrix
  Hj << (px/c2),               (py/c2),               0,     0,
       -(py/c1),               (px/c1),               0,     0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
