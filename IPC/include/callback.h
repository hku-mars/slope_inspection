#ifndef CALLBACK_H
#define CALLBACK_H

#include <ros/ros.h>

#include <mutex>
#include <tf/tf.h>
#include <Eigen/Eigen>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/PointCloud2.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "quadrotor_msgs/PositionCommand.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ipc {

enum UAV_mode_e {
    UAV_Manual    = 0,
    UAV_Hover     = 1,
    UAV_Pilot     = 2,
    UAV_AutoPilot = 3,
};

class CallbackClass {
public:
    typedef std::shared_ptr<CallbackClass> Ptr;
    CallbackClass(ros::NodeHandle& nh) {
        std::string node_name = ros::this_node::getName();
        LoadParam(nh, node_name);

        mode_          = UAV_Manual;
        odom_flag_     = false;
        // odom_flag_     = true; // just for debug
        new_goal_flag_ = false;
        new_rc_flag_   = false;
        arm_flag_      = false;
        rc_gain_       = 1.0;
        yaw_r_         = 0;
        odom_q_        = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        Gravity_       = Eigen::Vector3d(0, 0, 9.81);
        thrust_        = hover_perc_;
        thr2acc_       = Gravity_.z() / thrust_;
        odom_a_        = Eigen::Vector3d::Zero();

        reboot_srv_   = nh.serviceClient<mavros_msgs::CommandLong>("reboot");
        arm_srv_      = nh.serviceClient<mavros_msgs::CommandBool>("arming");
        mode_srv_     = nh.serviceClient<mavros_msgs::SetMode>("set_mode");

        pvaj_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("pvaj_cmd", 1);
        px4_cmd_pub_  = nh.advertise<mavros_msgs::AttitudeTarget>("px4_cmd", 1);
        astar_pub_    = nh.advertise<visualization_msgs::Marker>("astar_path", 1);
        ogm_pub_      = nh.advertise<sensor_msgs::PointCloud2>("astar_map", 1);
        mpc_path_pub_ = nh.advertise<nav_msgs::Path>("mpc_path", 1);
        goal_pub_     = nh.advertise<geometry_msgs::PoseStamped>("goal_pub", 1);
        sfc_pub_      = nh.advertise<visualization_msgs::MarkerArray>("sfc", 1);

        odom_sub_     = nh.subscribe<nav_msgs::Odometry>("odom", 10, &CallbackClass::OdomCallback, this, ros::TransportHints().tcpNoDelay());
        imu_sub_      = nh.subscribe<sensor_msgs::Imu>("imu", 10, &CallbackClass::IMUCallback, this, ros::TransportHints().tcpNoDelay());
        imu_raw_sub_  = nh.subscribe<sensor_msgs::Imu>("imu_raw", 10, &CallbackClass::IMURawCallback, this, ros::TransportHints().tcpNoDelay());
        battery_sub_  = nh.subscribe<sensor_msgs::BatteryState>("battery", 10, &CallbackClass::BatteryCallback, this, ros::TransportHints().tcpNoDelay());
        goal_sub_     = nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, &CallbackClass::GoalCallback, this, ros::TransportHints().tcpNoDelay());
        rc_sub_       = nh.subscribe<mavros_msgs::RCIn>("rc", 10, &CallbackClass::RCCallback, this, ros::TransportHints().tcpNoDelay());
        state_sub_    = nh.subscribe<mavros_msgs::State>("state", 10, &CallbackClass::StateCallback, this, ros::TransportHints().tcpNoDelay());

        init_time_ = ros::Time::now();
    }
    ~CallbackClass(){}

    inline bool SwitchArmedCmd(bool on_off) {
        if (on_off) {
            static ros::Time last_request;
            arm_cmd_.request.value = true;
            if (!uav_state_.armed && (ros::Time::now() - last_request).toSec() > 0.1) {
                if (arm_srv_.call(arm_cmd_) && arm_cmd_.response.success) ROS_INFO("Vehicle armed.");
                else {
                    ROS_INFO("\033[31mVehicle arming failed!\033[0m");
                    std::cout << "imu_acc: " << imu_a_.transpose() << std::endl;
                }
                last_request = ros::Time::now();
            }
        } else {
            static ros::Time last_request;
            arm_cmd_.request.value = false;
            if (!uav_state_.armed && (ros::Time::now() - last_request).toSec() > 0.1) {
                if (arm_srv_.call(arm_cmd_) && arm_cmd_.response.success) ROS_INFO("Vehicle disarmed.");
                else ROS_INFO("\033[31mVehicle disarming failed!\033[0m");
                last_request = ros::Time::now();
            }
        }
        return true;
    }
    inline void SetManualMode(void) {
        static ros::Time last_request;
        set_mode_.request.custom_mode = "Manual";
        if ((ros::Time::now() - last_request).toSec() > 1.0) {
            if (mode_srv_.call(set_mode_) && set_mode_.response.mode_sent) ROS_INFO("\033[41;37mManual enabled.\033[0m");
            else ROS_INFO("\033[31mSet Manual Mode failed!\033[0m");
        }
        last_request = ros::Time::now();
    }
    inline void SetOffboardMode(void) {
        static ros::Time last_request;
        set_mode_.request.custom_mode = "OFFBOARD";
        if ((ros::Time::now() - last_request).toSec() > 1.0) {
            if (mode_srv_.call(set_mode_) && set_mode_.response.mode_sent) ROS_INFO("\033[42;37mOffboard enabled.\033[0m");
            else ROS_INFO("\033[31mSet Offboard Mode failed!\033[0m");
        }
        last_request = ros::Time::now();
    }

    void PVAJCmdPublish(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r);
    void AttitudeCtrlPub(const Eigen::Quaterniond &q, const double thrust, const ros::Time &stamp);
    void BodyrateCtrlPub(const Eigen::Vector3d &rate, const double thrust, const ros::Time &stamp);
    void GoalPublish(Eigen::Vector3d goal, double yaw);
    void AstarPublish(std::vector<Eigen::Vector3d>& nodes, uint8_t type, double scale);
    void GridMapPublish(pcl::PointCloud<pcl::PointXYZ>& pc);
    void MPCPathPublish(std::vector<Eigen::Vector3d> &pt);

    ros::Publisher sfc_pub_;
    UAV_mode_e mode_;
    mavros_msgs::State uav_state_;
    sensor_msgs::BatteryState uav_battery_;
    Eigen::Vector3d Gravity_;

    Eigen::Vector3d odom_p_, odom_v_, odom_a_, imu_a_, imu_gyro_;
    Eigen::Quaterniond odom_q_;
    double yaw_, yaw_r_, yaw_dot_r_, rc_yaw_dot_;
    
    bool new_goal_flag_, new_rc_flag_, arm_flag_, odom_flag_;
    Eigen::Vector3d user_goal_, init_goal_, rc_joystick_;

    // param
    bool yaw_ctrl_flag_, simu_flag_, gazebo_flag_, bodyrate_flag_, hover_esti_flag_;
    double freq_;
    double sfc_dis_, vel_ref_, path_dis_;
    double rc_gain_, yaw_gain_, ctrl_delay_;
    double thrust_limit_, hover_perc_, thrust_, thr2acc_;
    Eigen::Vector3d map_size_;
    double resolution_, expand_fix_, expand_dyn_, update_dt_;

private:
    void LoadParam(ros::NodeHandle& nh, std::string& node_name) {
        nh.param(node_name + "/fsm/simulation", simu_flag_, false);
        nh.param(node_name + "/fsm/gazebo_flag", gazebo_flag_, false);
        nh.param(node_name + "/fsm/frequency", freq_, 100.0);
        nh.param(node_name + "/fsm/ctrl_delay", ctrl_delay_, 0.0);
        nh.param(node_name + "/fsm/sfc_dis", sfc_dis_, 0.0);

        nh.param(node_name + "/fsm/vel_ref", vel_ref_, 2.0);
        nh.param(node_name + "/fsm/path_dis", path_dis_, 0.1);

        nh.param(node_name + "/fsm/bodyrate_flag", bodyrate_flag_, true);
        nh.param(node_name + "/fsm/thrust_limit", thrust_limit_, 0.0);
        nh.param(node_name + "/fsm/hover_esti", hover_esti_flag_, true);
        nh.param(node_name + "/fsm/hover_perc", hover_perc_, 0.2);
        
        nh.param(node_name + "/fsm/yaw_ctrl_flag", yaw_ctrl_flag_, false);
        nh.param(node_name + "/fsm/yaw_gain", yaw_gain_, 0.5);

        nh.param(node_name + "/fsm/init_goal_x", init_goal_.x(), 0.0);
        nh.param(node_name + "/fsm/init_goal_y", init_goal_.y(), 0.0);
        nh.param(node_name + "/fsm/init_goal_z", init_goal_.z(), 0.5);
        user_goal_ = init_goal_;

        nh.param(node_name + "/astar/resolution", resolution_, 0.1);
        nh.param(node_name + "/astar/map_size_x", map_size_.x(), 5.0);
        nh.param(node_name + "/astar/map_size_y", map_size_.y(), 5.0);
        nh.param(node_name + "/astar/map_size_z", map_size_.z(), 3.0);
        nh.param(node_name + "/astar/expand_fix", expand_fix_, 0.1);
        nh.param(node_name + "/astar/expand_dyn", expand_dyn_, 0.1);
        nh.param(node_name + "/astar/update_dt", update_dt_, 2.0);
    }
    inline void Px4Reboot(void) {
        // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
        mavros_msgs::CommandLong srv;
        srv.request.broadcast = false;
        srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        srv.request.param1 = 1;	   // Reboot autopilot
        srv.request.param2 = 0;	   // Do nothing for onboard computer
        srv.request.confirmation = true;
        reboot_srv_.call(srv);
        ROS_INFO("\033[31m[PX4CTRL] Reboot PX4. Please restart mavros and this node! \033[31m");
    }

    void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void IMUCallback(const sensor_msgs::ImuConstPtr& msg);
    void IMURawCallback(const sensor_msgs::ImuConstPtr& msg);
    void GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void RCCallback(const mavros_msgs::RCInConstPtr& msg);
    void BatteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
        battery_mutex_.lock();
        uav_battery_ = *msg;
        if ((msg->header.stamp - init_time_).toSec() > 4.0 && simu_flag_ == false) {
            static int fps_count = 0;
            static ros::Time last_time;
            fps_count++;
            if ((msg->header.stamp - last_time).toSec() > 1.0) {
                last_time = msg->header.stamp;
                // if (fps_count < 40) ROS_ERROR("[IPC] Check topic: /mavros/battery. The frequency is %d.", fps_count);
                fps_count = 0;
            }
        }
        battery_mutex_.unlock();
    }
    void StateCallback(const mavros_msgs::StateConstPtr& msg) {
        state_mutex_.lock();
        uav_state_ = *msg;
        state_mutex_.unlock();
    }

    
    ros::Publisher     pvaj_cmd_pub_, px4_cmd_pub_, astar_pub_, ogm_pub_, mpc_path_pub_, goal_pub_;
    ros::Subscriber    odom_sub_, imu_sub_, imu_raw_sub_, battery_sub_, goal_sub_, rc_sub_, state_sub_;
    ros::ServiceClient reboot_srv_, arm_srv_, mode_srv_;
    ros::Time          init_time_, odom_time_, imu_time_, goal_time_, rc_time_;
    std::mutex         odom_mutex_, imu_mutex_, imu_raw_mutex_, battery_mutex_, goal_mutex_, rc_mutex_, state_mutex_;

    mavros_msgs::SetMode set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
};

}

#endif