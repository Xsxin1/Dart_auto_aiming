// 生成时间: 2026-05-09 18:55:02
// params.cpp
#include "ros2_hik_camera/params.h"
namespace Params {
    std::vector<ParamInfo<int>> int_params = {
{ "detector.loss_thres",  loss_thres,5},
{ "filter.filter_mode",  filter_mode,7},
{ "filter.bright.binary_thres",  binary_thres,100},
{ "filter.overexp.overexp_thres",  overexp_thres,240},
{ "filter.exg.exg_thres",  exg_thres,30},
};

    std::vector<ParamInfo<double>> double_params = {
{ "detector.conf_thres",  conf_thres,0.25},
{ "detector.nms_thres",  nms_thres,0.45},
{ "detector.dyaw_factor",  dyaw_factor,0.001},
{ "detector.filled_ratio",  filled_ratio,0.5},
{ "filter.green_ratio_thres",  green_ratio_thres,0.15},
};

    std::vector<ParamInfo<std::string>> string_params = {
{ "detector.weight_path",  weight_path,"./src/ros2_hik_camera/weight/light_320/best_openvino_model/best.xml"},
};

    std::vector<ParamInfo<bool>> bool_params = {
};

    std::vector<ParamInfo<std::vector<int>>> int_vec_params = {
};

    std::vector<ParamInfo<std::vector<double>>> double_vec_params = {
};

    std::vector<ParamInfo<std::vector<std::string>>> string_vec_params = {
};

    std::vector<ParamInfo<std::vector<bool>>> bool_vec_params = {
};
std::string weight_path= "./src/ros2_hik_camera/weight/light_320/best_openvino_model/best.xml";
double conf_thres= 0.25;
double nms_thres= 0.45;
double dyaw_factor= 0.001;
double filled_ratio= 0.5;
int loss_thres= 5;
int filter_mode= 7;
int binary_thres= 100;
int overexp_thres= 240;
int exg_thres= 30;
double green_ratio_thres= 0.15;

} // namespace Params
