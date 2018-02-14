#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {

	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
  // std::cout << "KalmanFilter::Predict():" << std::endl << x_ << std::endl;

}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

  UpdateWithY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  VectorXd z_pred = h(x_);
  VectorXd y = z - z_pred;
  // std::cout << "EKF update - Radar:" << std::endl;
  // std::cout << "EKF update - Radar - State:" << std::endl << x_ << std::endl;
  // std::cout << "EKF update - Radar - measurement:" << std::endl << z << std::endl;
  // std::cout << "EKF update - Radar - predict: " << std::endl << z_pred << std::endl;


  while (y[1] > M_PI) {
    y[1] -= 2 * M_PI;
  }

  while (y[1] < -M_PI) {
    y[1] += 2 * M_PI;
  }

  UpdateWithY(y);
}


VectorXd KalmanFilter::h(const VectorXd &x) {
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];

  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px); // [-pi, pi]
  double rho_dot = (px*vx + py*vy) / rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;

  return z_pred;
}

void KalmanFilter::UpdateWithY(const VectorXd &y) {

  // std::cout << "y:" << std::endl << y << std::endl;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  // New estimate based on new measurement update
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
