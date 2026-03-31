// 生成时间: 2026-03-31 16:06:11
// params.h
#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <vector>
#include <map>
#include <any>

namespace Params {

template<typename T>
struct ParamInfo {
    std::string name;
    T& storage_ref;
    T default_value;
};

extern std::vector<ParamInfo<int>> int_params;
extern std::vector<ParamInfo<double>> double_params;
extern std::vector<ParamInfo<std::string>> string_params;
extern std::vector<ParamInfo<bool>> bool_params;
extern std::vector<ParamInfo<std::vector<int>>> int_vec_params;
extern std::vector<ParamInfo<std::vector<double>>> double_vec_params;
extern std::vector<ParamInfo<std::vector<std::string>>> string_vec_params;
extern std::vector<ParamInfo<std::vector<bool>>> bool_vec_params;

extern std::string weight_path;
extern double conf_thres;
extern double nms_thres;
extern double dyaw_factor;
extern int binary_thres;
extern double filled_ratio;
extern int loss_thres;

                
} // namespace Params
#endif // PARAMS_H
