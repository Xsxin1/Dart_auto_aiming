import csv
import os
from pathlib import Path
from datetime import datetime

"""
fomat of csv:
namespace,type,name,default_value
"""

params = []
types_pairs = {'int':'int','double':'double','std::string':'string','bool':'bool','std::vector<int>':'int_vec','std::vector<double>':'double_vec','std::vector<std::string>':'string_vec','std::vector<bool>':'bool_vec'}
def read_params(config_file):
    if not os.path.exists(config_file):
        print(f"配置文件不存在: {config_file}")
        return
        
    with open(config_file,'r') as f:
        reader = csv.DictReader(f,delimiter=',',quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            if row['namespace'].startswith('#'):
                continue

            if row['type'] == 'std::string':
                if not row['default_value'].startswith('"'):
                    row['default_value'] = f'"{row["default_value"]}"'
            params.append(row)
    print(params)

def generate_header(output_dir):
    file=os.path.join(output_dir, 'params.h')
    with open(file, 'w') as f:
        f.write(f"""// 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
// params.h
#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <vector>
#include <map>
#include <any>

namespace Params {{

template<typename T>
struct ParamInfo {{
    std::string name;
    T& storage_ref;
    T default_value;
}};\n""")

        f.write('\n')
        for t, name in types_pairs.items():
            f.write(f"extern std::vector<ParamInfo<{t}>> {name}_params;\n")

        f.write('\n')
        for param in params:
            f.write(f"extern {param['type']} {param['name']};\n")

        f.write(f"""
                
}} // namespace Params
#endif // PARAMS_H
""")

        print(f"生成头文件: {os.path.join(output_dir, 'params.h')}")

def generate_source(output_dir,pkg_name):
    with open(os.path.join(output_dir, 'params.cpp'), 'w') as f:
        f.write(f"""// 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
// params.cpp
#include "{pkg_name}/params.h"
namespace Params {{""")
        
        for t, name in types_pairs.items():
            f.write(f"""
    std::vector<ParamInfo<{t}>> {name}_params = {{
""")
            for parm in params:
                if parm['type'] == t:
                    f.write(f"""{{ "{parm['namespace']}.{parm['name']}",  {parm['name']},{parm['default_value']}}},\n""")
            f.write(f"""}};\n""")   

        for param in params:
            f.write(f"{param['type']} {param['name']}= {param['default_value']};\n")

        f.write(f"""
}} // namespace Params
""")
        
        print(f"生成源文件: {os.path.join(output_dir, 'params.cpp')}")

if __name__ == '__main__':
    pkg_path=Path(__file__).parents[1]
    pkg_name = pkg_path.name
    ws_path=Path(__file__).parents[3]
    print(f"pkg_path: {pkg_path}"
          f"\nws_path: {ws_path}")
    read_params(f'{pkg_path}/config/params.csv')
    generate_header(f'{pkg_path}/include/{pkg_name}')
    generate_source(f'{pkg_path}/src',pkg_name)
                