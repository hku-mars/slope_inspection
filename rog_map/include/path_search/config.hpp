#pragma once

#include "ros/ros.h"
#include "vector"
#include "Eigen/Dense"
#include "string"
#include "type_utils/common_type_name.h"

namespace path_search {
    using namespace type_utils;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class PathSearchConfig {
    public:
        Vec3i map_voxel_num, map_size_i;
        bool visual_process;
        bool debug_visualization_en;
        bool allow_diag{false};
        int heu_type{0};

        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
                cout << param_value << endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                cout << param_value << endl;
                return false;
            }
        }

        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
                for (size_t i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                for (size_t i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return false;
            }
        }

        ros::NodeHandle nh_;

        PathSearchConfig() {};

        PathSearchConfig(const ros::NodeHandle &nh_priv, string name_space = "astar") {
            nh_ = nh_priv;
            vector<int> vox_;
            LoadParam(name_space + "/map_voxel_num", vox_, vox_);
            LoadParam(name_space + "/allow_diag", allow_diag, false);
            LoadParam(name_space + "/debug_visualization_en", debug_visualization_en, false);
            LoadParam(name_space + "/heu_type", heu_type, 0);
            LoadParam(name_space + "/visual_process", visual_process, false);
            map_voxel_num = Vec3i(vox_[0], vox_[1], vox_[2]);
            map_size_i = map_voxel_num / 2;
            map_voxel_num = map_size_i * 2 + Vec3i::Constant(1);
        }

    };
}
