#ifndef DETECTOR__PNP_SOLVER_HPP_
#define DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "detector/light.hpp"

namespace rm_auto_aim {
class PnPSolver {
 public:
  PnPSolver(const std::array<double, 9>& camera_matrix,
            const std::vector<double>& distortion_coefficients);

  // Get 3d position
  bool solvePnP(const Light& light, cv::Mat& rvec, cv::Mat& tvec);

  // Calculate the distance between light center and image center
  float calculateDistanceToCenter(const cv::Point2f& image_point);

 private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Unit: mm
  static constexpr float LIGHT_DIAMETER = 45;  //直径

  // Four vertices of light in 3d
  std::vector<cv::Point3f> light_points_;
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__PNP_SOLVER_HPP_
