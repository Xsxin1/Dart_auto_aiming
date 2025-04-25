#include "detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double radius = LIGHT_DIAMETER / 2.0 / 1000.0;

  // Start from  left in anti-clockwise order
  // Model coordinate: x forward , y left, z up
  light_points_ = {{0, 0, 0}, {0, -radius, 0}, {0, 0, radius}, {0, radius, 0}, {0, 0, -radius}};
}

bool PnPSolver::solvePnP(const Light & light, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_light_points;

  // Fill in image points
  image_light_points.emplace_back(light.center);
  image_light_points.emplace_back(light.rightPoint);
  image_light_points.emplace_back(light.topPoint);
  image_light_points.emplace_back(light.leftPoint);
  image_light_points.emplace_back(light.bottomPoint);

  // Solve pnp
  auto object_points = light_points_;
  return cv::solvePnP(
    object_points, image_light_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{  //cxcy为光心坐标
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
