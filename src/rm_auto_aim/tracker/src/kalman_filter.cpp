#include "tracker/kalman_filter.hpp"

namespace rm_auto_aim
{

KalmanFilter::KalmanFilter(
  const MatFunc & compute_F, const MatFunc & compute_H, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R, const Eigen::MatrixXd & P0)
: compute_F(compute_F),
  compute_H(compute_H),
  Q(Q),
  R(R),
  P_post(P0),
  n(compute_H(1).cols()),
  I(Eigen::MatrixXd::Identity(n, n))
{
  /*Todo:find a better way to initialize the matrices */
  // Initialize state vectors
  x_pri = Eigen::VectorXd::Zero(n);
  x_post = Eigen::VectorXd::Zero(n);
}

void KalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::VectorXd KalmanFilter::predict(double dt)
{
  // Compute the state transition matrix F
  F = compute_F(dt);
  // Compute the process noise covariance matrix Q
  H = compute_H(dt);

  // Predict the state
  x_pri = F * x_post;
  // Predict the error covariance
  P_pri = F * P_post * F.transpose() + Q;
  return x_pri;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::VectorXd & z)
{
  // Compute the Kalman gain
  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  // Update the state estimate
  x_post = x_pri + K * (z - H * x_pri);
  // Update the error covariance
  P_post = (I - K * H) * P_pri;
  return x_post;
}

}  // namespace rm_auto_aim