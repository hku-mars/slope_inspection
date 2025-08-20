#ifndef _TRAJ_OPT_CONFIG_
#define _TRAJ_OPT_CONFIG_

#include "vector"
#include "Eigen/Dense"
#include "string"
#include "type_utils/common_type_name.h"
#include "ros/ros.h"

namespace traj_opt {
using namespace type_utils;
using std::cout;
using std::endl;
using std::vector;
using std::string;
    enum PosConstrainType {
        WAYPOINT = 0,
        SPHERE = 1,
        POLYGON = 2
    };
    enum EnergyCostType {
        MIN_ACC = 2,
        MIN_JERK = 3,
        MIN_SNAP = 4
    };

    class TrajOptConfig {
    public:
        int pos_constraint_type;
        // 2 for min acc; 3 for min jerk, 4 for min snap
        int energy_cost_type;
        // Set to true for only min time.
        bool block_energy_cost;
        // Limit conditions.
        double max_vel, max_acc, max_jerk, max_omg, max_acc_thr, min_acc_thr;
        // Penalty cost.
        double penna_vel, penna_acc, penna_jerk, penna_omg, penna_min_acc_thr, penna_max_acc_thr;
        // penna_t; penna_pos only for corridor based method.
        double penna_t, penna_pos, penna_attract;
        // for time scale
        double scale_factor;
        // for backup traj piece num
        int piece_num;

		// for se3 traj opt, the size of drone
		double drone_max_len;
		double drone_min_len;

        double smooth_eps;
        int integral_reso;
        double opt_accuracy;

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
                for (int i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                for (int i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return false;
            }
        }

        ros::NodeHandle nh_;

        TrajOptConfig() {};

        TrajOptConfig(const ros::NodeHandle &nh_priv, string ns = "") {
            nh_ = nh_priv;
            if (ns == "") {
                ns = "/";
            } else {
                ns = "/" + ns + "/";
            }
            LoadParam("traj_opt" + ns + "pos_constraint_type", pos_constraint_type, 2);
            LoadParam("traj_opt" + ns + "energy_cost_type", energy_cost_type, 3);
            LoadParam("traj_opt" + ns + "piece_num", piece_num, 1);
            LoadParam("traj_opt" + ns + "block_energy_cost", block_energy_cost, false);
            LoadParam("traj_opt" + ns + "opt_accuracy", opt_accuracy, 1.0e-5);
            LoadParam("traj_opt" + ns + "scale_factor", scale_factor, 1.0);
            LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);
            LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);
		  	LoadParam("traj_opt" + ns + "drone_max_len", drone_max_len, -1.0);
		  	LoadParam("traj_opt" + ns + "drone_min_len", drone_min_len, -1.0);

            LoadParam("traj_opt" + ns + "max_vel", max_vel, -1.0);
            LoadParam("traj_opt" + ns + "max_acc", max_acc, -1.0);
            LoadParam("traj_opt" + ns + "max_jerk", max_jerk, -1.0);
            LoadParam("traj_opt" + ns + "max_omg", max_omg, -1.0);
            LoadParam("traj_opt" + ns + "max_acc_thr", max_acc_thr, -1.0);
            LoadParam("traj_opt" + ns + "min_acc_thr", min_acc_thr, -1.0);

            LoadParam("traj_opt" + ns + "penna_t", penna_t, -1.0);
            LoadParam("traj_opt" + ns + "penna_pos", penna_pos, -1.0);
            LoadParam("traj_opt" + ns + "penna_vel", penna_vel, -1.0);
            LoadParam("traj_opt" + ns + "penna_acc", penna_acc, -1.0);
            LoadParam("traj_opt" + ns + "penna_jerk", penna_jerk, -1.0);
            LoadParam("traj_opt" + ns + "penna_attract", penna_attract, -1.0);
            LoadParam("traj_opt" + ns + "penna_omg", penna_omg, -1.0);
            LoadParam("traj_opt" + ns + "penna_max_acc_thr", penna_max_acc_thr, -1.0);
            LoadParam("traj_opt" + ns + "penna_min_acc_thr", penna_min_acc_thr, -1.0);
        }

    };
}

#endif