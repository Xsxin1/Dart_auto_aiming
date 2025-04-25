#ifndef DETECTOR__LIGHT_HPP_
#define DETECTOR__LIGHT_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <chrono>
#include <string>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

enum class LightType { OUTPOST, BASE, INVALID };
const std::string Light_TYPE_STR[3] = {"outpost", "base", "invalid"};

struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box, std::vector<cv::Point> ellipsePoints)
  : cv::RotatedRect(box), ellipsePoints(ellipsePoints)
  {
    rightPoint = cv::Point2f(box.center.x + box.size.width / 2, box.center.y);
    leftPoint = cv::Point2f(box.center.x - box.size.width / 2, box.center.y);
    topPoint = cv::Point2f(box.center.x, box.center.y + box.size.height / 2);
    bottomPoint = cv::Point2f(box.center.x, box.center.y - box.size.height / 2);
  }

  std::vector<cv::Point> ellipsePoints;
  cv::Point2f rightPoint;
  cv::Point2f leftPoint;
  cv::Point2f topPoint;
  cv::Point2f bottomPoint;

  //in camera coordinate from the camera center to the light center
  float distance;  //m
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__LIGHT_HPP_
