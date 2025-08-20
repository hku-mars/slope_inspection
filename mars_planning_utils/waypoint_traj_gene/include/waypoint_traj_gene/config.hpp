//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_CONFIG
#define MISSION_PLANNER_CONFIG

#include "ros/ros.h"
#include "vector"
#include "string"
#include "ros/package.h"
#include "type_utils/common_type_name.h"
#include "traj_opt/config.hpp"

using namespace traj_opt;
using namespace type_utils;

namespace waypoint_planner {

    enum TriggerType {
        RVIZ_CLICK = 0,
        MAVROS_RC = 1,
        TARGET_ODOM = 2
    };

    class MissionConfig {
    public:
        // Bool Params

        TrajOptConfig opt_cfg;

        int start_trigger_type;

        string path_pub_topic, goal_pub_topic, odom_topic, waypoints_file_name;
        Mat3Df waypoints;
        Eigen::Matrix<double, 1, -1> yaw_waypoints;
        double switch_dis;
        double odom_timeout;
        double publish_dt;
        double scale;
        double height;
        double yaw_dot_max;
        // yaw mode 1 for heading velocity, yaw mode 2 for given waypoint.
        int yaw_mode;

        double str2double(string s) {
            double d;
            std::stringstream ss;
            ss << s;
            ss >> std::setprecision(16) >> d;
            ss.clear();
            return d;
        }

        void LoadWaypoint(string file_name) {
            string package_path_ = ros::package::getPath("waypoint_traj_gene");

            file_name = package_path_ + "/data/" + file_name;
            std::ifstream theFile(file_name);
            std::string line;
            Vec3f log;
            vector<double> yaws;
            vec_Vec3f waypts;
            while (std::getline(theFile, line)) {
                std::vector<std::string> result;
                std::istringstream iss(line);
                for (std::string s; iss >> s;) {
                    result.push_back(s);
                }
                for (size_t i = 0; i < 3; i++) {
                    log(i) = str2double(result[i]);
                }
                if (result.size() == 4) {
                    yaws.push_back(str2double(result[3]));
                }
                log = log * scale;
                if (height > 0) {
                    log(2) = height;
                }
                waypts.push_back(log);
            }
            if (!yaws.empty()) {
                yaw_waypoints.resize(yaws.size());
                for (int i = 0; i < yaws.size(); i++) {
                    yaw_waypoints(i) = yaws[i];
                }
            }
            if (!waypts.empty()) {
                waypoints.resize(3, waypts.size());
                for (int i = 0; i < waypts.size(); i++) {
                    waypoints.col(i) = waypts[i];
                }
            }
        }

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

        MissionConfig() {};

        MissionConfig(const ros::NodeHandle &nh_priv) {
            nh_ = nh_priv;
            LoadParam("mission_planner/scale", scale, 1.0);
            LoadParam("mission_planner/height", height, 1.0);
            LoadParam("mission_planner/start_trigger_type", start_trigger_type, 1);
            LoadParam("mission_planner/yaw_dot_max", yaw_dot_max, 1.0);
            LoadParam("mission_planner/yaw_mode", yaw_mode, 1);
            LoadParam("mission_planner/switch_dis", switch_dis, 1.0);
            LoadParam("mission_planner/odom_timeout", odom_timeout, 0.1);
            LoadParam("mission_planner/publish_dt", publish_dt, 1.0);
            LoadParam("mission_planner/goal_pub_topic", goal_pub_topic, string("/planner/goal"));
            LoadParam("mission_planner/odom_topic", odom_topic, string("/lidar_slam/odom"));
            LoadParam("mission_planner/path_pub_topic", path_pub_topic, string("/planner/path_cmd"));
            LoadParam("mission_planner/waypoints_file_name", waypoints_file_name, string("a_working_waypoints.txt"));

            LoadWaypoint(waypoints_file_name);

            opt_cfg = TrajOptConfig(nh_priv, "way_pt");

            cout << GREEN " -- [MISSION] Load " << waypoints.size() << " waypoints success." << RESET << endl;
            for (int i = 0; i < waypoints.cols(); i++) {
                cout << BLUE << "\t Waypoint " << i << " at [" << waypoints.col(i).transpose() << "]" << RESET << endl;
                if (!(yaw_waypoints.size() == 0)) {
                    cout << BLUE << "\t Yaw " << i << " at " << yaw_waypoints[i] << RESET << endl;
                }
            }
            string types[]{"RVIZ_CLICK", "MAVROS_RC"};
            string way_pt_types[]{"PATH", "WAYPOINTS"};
            cout << YELLOW << " -- [MISSION] Trigger type " << types[start_trigger_type] << RESET << endl;

        }

    };
}
#endif //PLANNER_CONFIG_HPP
