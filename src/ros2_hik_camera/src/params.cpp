// 生成时间: 2026-03-29 19:44:30
// params.cpp
#include "ros2_hik_camera/params.h"
namespace Params {
    std::vector<ParamInfo<int>> int_params = {
};

    std::vector<ParamInfo<double>> double_params = {
{ "detector.conf_thres",  conf_thres,0.25},
{ "detector.nms_thres",  nms_thres,0.45},
{ "detector.dyaw_factor",  dyaw_factor,0.001},
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

} // namespace Params
