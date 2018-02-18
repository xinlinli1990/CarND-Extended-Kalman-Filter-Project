#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement function H - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // measurement function H - radar
  // H_radar = Hj(x)

  // Initialize covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
  		     0, 1, 0, 0,
           0, 0, 1000, 0,
           0, 0, 0, 1000;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  // cout << "Raw input" <<endl;
  // if (measurement_pack.raw_measurements_.size() == 3) {
  //     float rho = measurement_pack.raw_measurements_[0];
  //     float phi = measurement_pack.raw_measurements_[1];
  //     float rho_dot = measurement_pack.raw_measurements_[2];
  //     cout << "rho: " << rho << endl;
  //     cout << "phi: " << phi << endl;
  //     cout << "rho_dot: " << rho_dot << endl;      
  // } else {
  //     float p_x = measurement_pack.raw_measurements_[0];
  //     float p_y = measurement_pack.raw_measurements_[1];
  //     cout << "p_x: " << p_x << endl;
  //     cout << "p_y: " << p_y << endl;
  // }



  if (!is_initialized_) {
    // Initialize state x
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		double rho = measurement_pack.raw_measurements_[0];
		double phi = measurement_pack.raw_measurements_[1];
		double rho_dot = measurement_pack.raw_measurements_[2];

		double px = rho * cos(phi);
		double py = rho * sin(phi);
		double vx = rho_dot * sin(phi);
		double vy = rho_dot * cos(phi);

		ekf_.x_ << px, py, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		double p_x = measurement_pack.raw_measurements_[0];
		double p_y = measurement_pack.raw_measurements_[1];
		double v_x = 0.0;
		double v_y = 0.0;

		ekf_.x_ << p_x, p_y, v_x, v_y;
    }

	previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // cin.get();

  double delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // cout << "measurement_pack.timestamp_" << measurement_pack.timestamp_ << endl;
  // cout << "previous_timestamp_" << previous_timestamp_ << endl;
  // cout << "delta_t: " << delta_t << endl;

  // Compute efk_.F_ - based on delta_t
  ekf_.F_ = MatrixXd::Identity(4, 4);
  ekf_.F_(0, 2) = delta_t;
  ekf_.F_(1, 3) = delta_t;

  double noise_ax = 9.0;
  double noise_ay = 9.0;

  // Compute efk_.Q_ - based on delta_t
  double dt2 = delta_t * delta_t; // dt^2
  double dt3 = dt2 * delta_t;     // dt^3
  double dt4 = dt3 * delta_t;     // dt^4
  double dt3_2 = 0.5 * dt3;       // dt^3 / 2
  double dt4_4 = 0.25 * dt4;      // dt^4 / 4

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4_4 * noise_ax, 0,                 dt3_2 * noise_ax, 0,
             0,                dt4_4 * noise_ay,  0,                dt3_2 * noise_ay,
             dt3_2 * noise_ax, 0,                 dt2 * noise_ax,   0,
             0,                dt3_2 * noise_ay,  0,                dt2 * noise_ay;

  // cout << "Before predict" << endl;
  // cout << "ekf_.F_: " << endl << ekf_.F_ << endl;
  // cout << "ekf_.Q_: " << endl << ekf_.Q_ << endl;
  // cout << "ekf_.x_: " << endl << ekf_.x_ << endl;
  // cout << "ekf_.P_: " << endl << ekf_.P_ << endl;

  if (delta_t > 0.0000001) {
    ekf_.Predict();
  }

  // cout << "After predict" << endl;
  // cout << "ekf_.x_: " << endl << ekf_.x_ << endl;
  // cout << "ekf_.P_: " << endl << ekf_.P_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.R_ = R_radar_;
    // cout << "Radar Update - measurement:" << endl << measurement_pack.raw_measurements_ << endl;
    // cout << "ekf_.H_:" << endl << ekf_.H_ << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
    // cout << "Laser Update - measurement:" << endl << measurement_pack.raw_measurements_ << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << endl << ekf_.x_ << endl;
  cout << "P_ = " << endl << ekf_.P_ << endl;

}
