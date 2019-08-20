#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  // Prediction based on linear motion
  x_ = F_ * x_;                     // s = vt
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;   // Q captures acceleration, uncertainty of motion
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;        // drop velocity component of x
  VectorXd y = z - z_pred;          // calc difference

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;   // R uncertainty of measurement
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si; // kalman filter gain


  // new estimate based on measurement
  x_ = x_ + (K * y);

  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_; // update P matrix
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Recover explicitly status information
  float px = x_(0); 
  float py = x_(1);
  float vx = x_(2); 
  float vy = x_(3);

  // Map predicted state into measurement space
  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / std::max(rho, 0.0001);
  VectorXd z_pred(3); z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;
  while (y(1) > M_PI)  {y(1) -= 2 * M_PI;}
  while (y(1) < -M_PI) {y(1) += 2 * M_PI;}

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; // R uncertainty of measurement
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si; // kalman filter gain

  // Update estimate
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}
