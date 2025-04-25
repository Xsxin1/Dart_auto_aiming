#ifndef TRACKER__KALMAN_FILTER_HPP_
#define TRACKER__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

class KalmanFilter
{
public:
  KalmanFilter() = default;

  // Define function types for dynamic computation
  using MatFunc = std::function<Eigen::MatrixXd(double)>;  // Function to compute F or Q based on dt

  explicit KalmanFilter(
    const MatFunc & compute_F, const MatFunc & compute_H, const Eigen::MatrixXd & Q,
    const Eigen::MatrixXd & R, const Eigen::MatrixXd & P0);

  void setState(const Eigen::VectorXd & x0);

  Eigen::VectorXd predict(double dt);

  Eigen::VectorXd update(const Eigen::VectorXd & z);

private:
  // Function to compute the state transition matrix F based on dt
  MatFunc compute_F;
  Eigen::MatrixXd F;
  // Function to compute the process noise covariance matrix Q based on dt
  MatFunc compute_H;
  Eigen::MatrixXd H;
  // Observation matrix
  Eigen::MatrixXd Q;
  // Measurement noise covariance matrix
  Eigen::MatrixXd R;

  // Priori error estimate covariance matrix
  Eigen::MatrixXd P_pri;
  // Posteriori error estimate covariance matrix
  Eigen::MatrixXd P_post;

  // Kalman gain
  Eigen::MatrixXd K;

  // System dimensions
  int n;

  // N-size identity matrix
  Eigen::MatrixXd I;

  // Priori state
  Eigen::VectorXd x_pri;
  // Posteriori state
  Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // TRACKER__KALMAN_FILTER_HPP_