#include "ros2_hik_camera/openvino.h"

YOLO_OPENVINO::YOLO_OPENVINO() {}

YOLO_OPENVINO::~YOLO_OPENVINO() {}

YOLO_OPENVINO::Resize YOLO_OPENVINO::resize_and_pad(cv::Mat & img, cv::Size new_shape)
{
  float width = img.cols;
  float height = img.rows;
  float r = float(new_shape.width / max(width, height));
  int new_unpadW = int(round(width * r));
  int new_unpadH = int(round(height * r));

  cv::resize(img, resize.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, cv::INTER_LINEAR);

  resize.dw = new_shape.width - new_unpadW;   // w方向padding值
  resize.dh = new_shape.height - new_unpadH;  // h方向padding值

  // cv::Scalar color = cv::Scalar(100, 100, 100);
  // cv::copyMakeBorder(resize.resized_image, resize.resized_image, 0,
  // resize.dh,
  //                    0, resize.dw, cv::BORDER_CONSTANT, color);
  // 使用更高效的边界填充方式
  if (resize.dh > 0 || resize.dw > 0) {
    cv::copyMakeBorder(
      resize.resized_image, resize.resized_image, 0, resize.dh, 0, resize.dw, cv::BORDER_CONSTANT,
      cv::Scalar(114, 114, 114));  // YOLO常用的填充值
  }

  return resize;
}

void YOLO_OPENVINO::yolov5_compiled(std::string xml_path, ov::CompiledModel & compiled_model)
{
  // Step 1. Initialize OpenVINO Runtime core
  static ov::Core core;

  // 检查GPU设备是否可用
  std::vector<std::string> available_devices = core.get_available_devices();
  bool has_gpu =
    std::find(available_devices.begin(), available_devices.end(), "GPU") != available_devices.end();

  // Step 2. Read a model
  std::shared_ptr<ov::Model> model = core.read_model(xml_path);

  // Step 4. Inizialize Preprocessing for the model 初始化模型的预处理
  ov::preprocess::PrePostProcessor ppp(model);
  // Specify input image format 指定输入图像格式
  ppp.input()
    .tensor()
    .set_element_type(ov::element::u8)
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);
  // Specify preprocess pipeline to input image without resizing
  // 指定输入图像的预处理管道而不调整大小
  ppp.input()
    .preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale({255., 255., 255.});
  //  Specify model's input layout 指定模型的输入布局
  ppp.input().model().set_layout("NCHW");
  // Specify output results format 指定输出结果格式
  ppp.output().tensor().set_element_type(ov::element::f32);
  // Embed above steps in the graph 在图形中嵌入以上步骤
  model = ppp.build();

  // Step 4. Compile model for GPU with optimizations
  if (has_gpu) {
    // GPU特定配置
    ov::AnyMap gpu_config = {
      {"PERFORMANCE_HINT", "LATENCY"},
      {"MODEL_PRIORITY", "HIGH"},
      {"GPU_ENABLE_LOOP_UNROLLING", "YES"}};
    compiled_model = core.compile_model(model, "GPU", gpu_config);
    std::cout << "Using " << available_devices[1] << std::endl;
  } else {
    // 回退到CPU配置
    std::cout << "GPU not available, falling back to CPU" << std::endl;
    ov::AnyMap cpu_config = {
      {"PERFORMANCE_HINT", "THROUGHPUT"}, {"INFERENCE_PRECISION_HINT", "f16"}};
    compiled_model = core.compile_model(model, "CPU", cpu_config);
  }
}

cv::Rect YOLO_OPENVINO::yolov5_detector(
  ov::CompiledModel compiled_model, cv::Mat & input_detect_img, cv::Mat & output_detect_img,
  vector<cv::Rect> & nms_box, vector<int> & nms_confidence, float size = 640.0,
  float conf_thres = 0.4, float nms_thres = 0.4)
{
  cv::Mat & img = input_detect_img;
  vector<cv::Rect> boxes;
  vector<int> class_ids;
  vector<float> confidences;
  cv::Rect box;

  // 预分配内存
  boxes.reserve(1000);
  class_ids.reserve(1000);
  confidences.reserve(1000);

  // 直接处理单张图像
  Resize res = resize_and_pad(img, cv::Size(size, size));

  // 创建输入张量
  ov::Tensor input_tensor(
    compiled_model.input().get_element_type(), compiled_model.input().get_shape(),
    res.resized_image.data);

  // 推理
  static ov::InferRequest infer_request = compiled_model.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // 处理输出
  const ov::Tensor & output_tensor = infer_request.get_output_tensor();
  float * detections = output_tensor.data<float>();
  ov::Shape output_shape = output_tensor.get_shape();

  // 后处理
  for (int i = 0; i < output_shape[1]; i++) {
    float * detection = &detections[i * output_shape[2]];
    float confidence = detection[4];

    if (confidence >= conf_thres) {
      float * classes_scores = &detection[5];
      cv::Mat scores(1, output_shape[2] - 5, CV_32FC1, classes_scores);
      cv::Point class_id;
      double max_class_score;
      cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

      if (max_class_score > SCORE_THRESHOLD) {
        confidences.push_back(confidence);
        class_ids.push_back(class_id.x);

        // 坐标转换
        float x = detection[0];
        float y = detection[1];
        float w = detection[2];
        float h = detection[3];

        float rx = (float)img.cols / (res.resized_image.cols - res.dw);
        float ry = (float)img.rows / (res.resized_image.rows - res.dh);

        x = rx * x;
        y = ry * y;
        w = rx * w;
        h = ry * h;

        boxes.push_back(cv::Rect(x - w / 2, y - h / 2, w, h));
      }
    }
  }

  // NMS处理
  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, nms_thres, nms_result);

  // 输出结果
  nms_box.clear();
  nms_box.reserve(nms_result.size());
  for (int idx : nms_result) {
    nms_box.push_back(boxes[idx]);
    nms_confidence.push_back(confidences[idx]);
  }

  // 绘制结果
  img.copyTo(output_detect_img);
  for (int i = 0; i < nms_result.size(); i++) {
    int idx = nms_result[i];
    cv::Rect box = boxes[idx];
    cv::rectangle(output_detect_img, box, cv::Scalar(0, 255, 0), 2);
    cv::putText(
      output_detect_img, to_string(confidences[idx]), box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 0, 255), 2);
  }
  if (confidences.size() > 0) {
    int max_conf_index = *std::max_element(confidences.begin(), confidences.end());
    box = boxes[max_conf_index];
  }

  return box;
}