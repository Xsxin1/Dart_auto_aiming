#pragma once
#pragma once
#pragma once
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

#include "params.h"

using namespace std;

class YOLO_OPENVINO
{
public:
  YOLO_OPENVINO();
  ~YOLO_OPENVINO();

public:
  struct Detection
  {
    int class_id;
    float confidence;
    cv::Rect box;
  };

  struct Resize
  {
    cv::Mat resized_image;
    int dw;
    int dh;
  };

  Resize resize_and_pad(cv::Mat & img, cv::Size new_shape);
  void yolov5_compiled(std::string xml_path, ov::CompiledModel & compiled_model);
  cv::Rect yolov5_detector(
    ov::CompiledModel compiled_model, cv::Mat & input_detect_img, cv::Mat & output_detect_img,
    vector<cv::Rect> & nms_box, float size, float conf_thres, float nms_thres);

private:
  const float SCORE_THRESHOLD = 0.7;
  Resize resize;
};