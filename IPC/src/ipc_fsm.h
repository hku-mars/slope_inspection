#ifndef IPC_FSM_H
#define IPC_FSM_H

#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <queue>
#include <sensor_msgs/PointCloud2.h>

#include "../include/astar_rm.h"
#include "../include/callback.h"
#include "../include/ciri.h"
#include "../include/emvp.h"
#include "../include/mpc.h"

namespace ipc {

class IPCFSMClass {
public:
    IPCFSMClass(ros::NodeHandle& nh) {
        std::string node_name = ros::this_node::getName();
        nh.param(node_name + "/fsm/esti_R", esti_R_, 0.3);
        nh.param(node_name + "/fsm/esti_log_flag", esti_log_flag_, false);

        nh.param(node_name + "/fsm/pilot_vw", pilot_vw_, 1.0);
        pilot_vw_pilot_ = pilot_vw_;
        nh.param(node_name + "/fsm/pilot_vx", pilot_v_.x(), 1.0);
        nh.param(node_name + "/fsm/pilot_vy", pilot_v_.y(), 1.0);
        nh.param(node_name + "/fsm/pilot_vz", pilot_v_.z(), 1.0);
        nh.param(node_name + "/fsm/auto_pilot_vx", auto_pilot_v_.x(), 1.0);
        nh.param(node_name + "/fsm/auto_pilot_vy", auto_pilot_v_.y(), 1.0);
        nh.param(node_name + "/fsm/auto_pilot_vz", auto_pilot_v_.z(), 1.0);
        
        esti_P_ = 100.0;
        
        arm_flag_ = false;
        replan_flag_ = false;
        replan_cnt_ = 0;
        astar_index_ = 0;
        delay_ctrl_time_ = ros::Time::now();

        cb_  = std::make_shared<CallbackClass>(nh);
        mpc_ = std::make_shared<MPCClass>(nh);
        rog_map::ROGMapConfig rog_map_cfg;
        rog_map::ROSParamLoader(nh, rog_map_cfg, node_name + "/rog_map");
        map_ptr_.reset(new rog_map::ROGMap(nh, rog_map_cfg));
        astar_ = std::make_shared<AstarClass>();
        astar_->InitMap(map_ptr_->getCfg().inflation_resolution, Eigen::Vector3d(map_ptr_->getCfg().map_size_d(0), map_ptr_->getCfg().map_size_d(1), map_ptr_->getCfg().map_size_d(2)), map_ptr_);
        double robot_r = 0.35;
        double iter_num = 1;
        resolution_ = map_ptr_->getCfg().inflation_resolution;
        // std::cout << "resolution_: " << resolution_ << std::endl;
        ciri_.setupParams(robot_r, iter_num);

        // connect to flight controller (FMU)
        if (cb_->simu_flag_) { // marsim and gazebo simulation
            if (cb_->gazebo_flag_) { // for gazebo simulation
                ROS_INFO("\033[36m[IPC] Gazebo simulation.");
                GazeboSimulationInit();
            } else {
                ROS_INFO("\033[36m[IPC] Marsim simulation.");
                cb_->mode_ = UAV_Hover;
            }
            ROS_INFO("\033[32m[IPC] Simulation On!");
        } else { // for real world px4 flight controller (FMU)
            ROS_INFO("\033[36m[IPC] Real world experiments.");
            PX4CheckConnection();
        }

        ros::Rate(2).sleep();

        local_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("local_pc", 10, &IPCFSMClass::LocalPcCallback, this, ros::TransportHints().tcpNoDelay());
        odom_free_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/ipc/odom_free", 10);
        goal_free_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/ipc/goal_free", 10);
        timer_ = nh.createTimer(ros::Duration(1.0/cb_->freq_), &IPCFSMClass::TimerCallback, this, false, true);
        TimeLogInit(); // initial log file, must behind timer initialization
    }
    ~IPCFSMClass() {
        log_file_.close();
    }

private:
    bool estimateThrustModel(const Eigen::Vector3d &est_a) {
        if (cb_->hover_esti_flag_ == false) {
            cb_->thr2acc_ = cb_->Gravity_.z() / cb_->hover_perc_;
            return true;
        }
        if (cb_->mode_ == UAV_Manual) {
            esti_P_ = 100.0;
            cb_->thr2acc_ = cb_->Gravity_.z() / cb_->hover_perc_;
            return true;
        }
        ros::Time t_now = ros::Time::now();
        if (time_thrust_.size() == 0) return false;
        std::pair<ros::Time, double> t_t = time_thrust_.front();

        while (time_thrust_.size() >= 1) {
            double delta_t = (t_now - t_t.first).toSec();
            if (delta_t > 1.0) {
                time_thrust_.pop();
                continue;
            } 
            if (delta_t < 0.035) {
                return false;
            }

            /* Recursive least squares algorithm with vanishing memory */
            double thr = t_t.second;
            time_thrust_.pop();
            /* Model: est_a(2) = thr2acc * thr */
            double K = esti_P_ / (esti_P_ + esti_R_); // using Kalman filter
            cb_->thr2acc_ = cb_->thr2acc_ + K * (est_a(2) - thr * cb_->thr2acc_);
            esti_P_ = (1 - K * thr) * esti_P_;
            double hover_percentage = cb_->Gravity_.z() / cb_->thr2acc_;
            if (hover_percentage > 0.8 || hover_percentage < 0.1) {
                ROS_INFO_THROTTLE(1, "Estimated hover_percentage > 0.8 or < 0.1! Perhaps the accel vibration is too high!");
                cb_->thr2acc_ = hover_percentage > 0.8 ? cb_->Gravity_.z() / 0.8 : cb_->thr2acc_;
                cb_->thr2acc_ = hover_percentage < 0.1 ? cb_->Gravity_.z() / 0.1 : cb_->thr2acc_;
            }
            if (esti_log_flag_) {
                static ros::Time last_time;
                if ((t_now - last_time).toSec() > 1.0) {
                    last_time = t_now;
                    ROS_WARN("[IPC] hover_percentage = %f", hover_percentage);
                }
            }
            return true;
        }
        return false;
    }
    void ComputeThrust(Eigen::Vector3d acc) {
        const Eigen::Vector3d zB = cb_->odom_q_ * Eigen::Vector3d::UnitZ();
        double des_acc_norm = acc.dot(zB);
        cb_->thrust_ = des_acc_norm / cb_->thr2acc_;
    }
    void ConvertCommand(Eigen::Vector3d acc, Eigen::Vector3d jerk) {
        Eigen::Vector3d xB, yB, zB, xC;
        if (cb_->yaw_ctrl_flag_) {
            double yaw_error = cb_->yaw_r_ - cb_->yaw_;
            if (yaw_error >  M_PI) yaw_error -= M_PI * 2;
            if (yaw_error < -M_PI) yaw_error += M_PI * 2;
            cb_->yaw_dot_r_ = yaw_error * cb_->yaw_gain_;
        } else {
            cb_->yaw_dot_r_ = (0 - cb_->yaw_) * cb_->yaw_gain_;
        }
        if (cb_->bodyrate_flag_) xC << std::cos(cb_->yaw_), std::sin(cb_->yaw_), 0;
        else xC << std::cos(cb_->yaw_r_), std::sin(cb_->yaw_r_), 0;

        zB = acc.normalized();
        yB = (zB.cross(xC)).normalized();
        xB = yB.cross(zB);
        Eigen::Matrix3d R;
        R << xB, yB, zB;
        u_q_ = R;

        Eigen::Vector3d hw = (jerk - (zB.dot(jerk) * zB)) / acc.norm();
        u_w_.x() = -hw.dot(yB);
        u_w_.y() = hw.dot(xB);
        u_w_.z() = cb_->yaw_dot_r_ * zB.dot(Eigen::Vector3d(0, 0, 1));
    }
    void YawCtrl(void) {
        
    }
    inline void PX4CheckConnection(void) {
        while (ros::ok() && !cb_->uav_state_.connected) {
            static ros::Time last_time;
            ros::spinOnce();
            ros::Time now_time = ros::Time::now();
            if ((now_time - last_time).toSec() > 2.0) {
                last_time = now_time;
                ROS_WARN("[IPC] Waiting for PX4 connection.");
            }
            ros::Rate(20).sleep();
        }
        ROS_INFO("\033[32m[IPC] PX4 FMU connect!");
    }
    inline void GazeboSimulationInit(void) {
        PX4CheckConnection();
        while (ros::ok() && cb_->uav_state_.mode != "OFFBOARD") { // set offboard mode
            ros::spinOnce();
            cb_->BodyrateCtrlPub(Eigen::Vector3d(0,0,0), 0, ros::Time::now());
            cb_->SetOffboardMode();
            ros::Time now_time = ros::Time::now();
            ros::Rate(20).sleep();
        }

        cb_->mode_ = UAV_Hover; // set to Hover mode
        cb_->user_goal_ = cb_->init_goal_ + cb_->odom_p_;
        cb_->yaw_r_ = cb_->yaw_;
    }
    void TimeLogInit(void) {
        // initial log file, must behind timer initialization
        static bool init_flag = false;
        if (init_flag) return;
        init_flag = true;
        std::string file = ros::package::getPath("ipc") + "/config/time.csv";
        log_file_.open(file, std::ios::out | std::ios::trunc);
        log_times_.resize(7, 1);
        log_file_ << "fsm" << ", " << "mapping" << ", " << "replan" << ", " << "sfc" << ", " << "mpc" << ", " << "df" << ", " << "total" << ", " << std::endl;
        for (int i = 0; i < log_times_.size(); i++) {
            log_times_[i] = 0;
        }
    }
    void TimeLogWrite(void) {
        if (cb_->mode_ == UAV_Manual) return ;
        for (int i = 0; i < log_times_.size(); i++) {
            log_file_ << log_times_[i] << ", ";
            log_times_[i] = 0.0;
        }
        log_file_ << std::endl;
    }

    void LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        odom_mutex_.lock();
        if (cb_->odom_flag_ == false) {
            odom_mutex_.unlock();
            return;
        }
        odom_p_pilot_ = cb_->odom_p_;
        odom_v_pilot_ = cb_->odom_v_;
        odom_a_pilot_ = cb_->odom_a_;
        odom_q_pilot_ = cb_->odom_q_;
        user_goal_pilot_ = cb_->user_goal_;
        yaw_pilot_ = cb_->yaw_;
        yaw_r_pilot_ = cb_->yaw_r_;
        new_rc_flag_ = cb_->new_rc_flag_;
        cb_->new_rc_flag_ = false;
        rc_yaw_dot_pilot_ = cb_->rc_yaw_dot_;
        rc_gain_pilot_ = cb_->rc_gain_;
        rc_joystick_pilot_ = cb_->rc_joystick_;

        double dis_min = 10000.0;
        sfc_seed_ = odom_p_pilot_;
        for (int i = 0; i < ref_path_.size(); i++) {
            double dis = (ref_path_[i] - odom_p_pilot_).norm();
            if (dis < dis_min) {
                dis_min = dis;
                sfc_seed_ = ref_path_[i];
            }
        }
        odom_mutex_.unlock();
        
        local_pc_mutex_.lock();
        ros::Time t0 = ros::Time::now();
        pcl::fromROSMsg(*msg, cloud_);

        type_utils::Pose p;
        p.first.x()  = odom_p_pilot_.x();
        p.first.y()  = odom_p_pilot_.y();
        p.first.z()  = odom_p_pilot_.z();
        p.second.w() = odom_q_pilot_.w();
        p.second.x() = odom_q_pilot_.x();
        p.second.y() = odom_q_pilot_.y();
        p.second.z() = odom_q_pilot_.z();
        map_ptr_->updateMap(cloud_, p);
        static ros::Time vis_time;
        if ((t0 - vis_time).toSec() > 0.1) {
            vis_time = t0;
            map_ptr_->viz();
        }

        // if (cb_->mode_ == UAV_AutoPilot) std::cout << "boxsearch start!" << std::endl;
        Eigen::Vector3d box_max(1.0, 1.0, 0.8), box_min(-1.0, -1.0, -0.8);
        pc_occ_.clear();
        pc_unk_.clear();
        pc_.clear();
        map_ptr_->boxSearch(odom_p_pilot_+box_min, odom_p_pilot_+box_max, rog_map::OCCUPIED, pc_occ_);
        map_ptr_->boxSearch(odom_p_pilot_+box_min, odom_p_pilot_+box_max, type_utils::FRONTIER, pc_unk_);
        // map_ptr_->boxSearch(odom_p_pilot_+box_min, odom_p_pilot_+box_max, rog_map::UNKNOWN, pc_unk_);
        // // if (pc_occ_.size() > 0) pc_.insert(pc_.end(), pc_occ_.begin(), pc_occ_.end());
        // // if (pc_unk_.size() > 0) pc_.insert(pc_.end(), pc_unk_.begin(), pc_unk_.end());
        if (pc_occ_.size() > 0 || pc_unk_.size() > 0) pc_.resize(pc_occ_.size() + pc_unk_.size());
        if (pc_occ_.size() > 0) std::copy(pc_occ_.begin(), pc_occ_.end(), pc_.begin());
        if (pc_unk_.size() > 0) std::copy(pc_unk_.begin(), pc_unk_.end(), pc_.begin() + pc_occ_.size());
        pc_rs_.resize(pc_.size());
        if (pc_occ_.size() > 0) std::fill(pc_rs_.begin(), pc_rs_.begin() + pc_occ_.size(), 0.35);
        if (pc_unk_.size() > 0) std::fill(pc_rs_.begin() + pc_occ_.size(), pc_rs_.begin() + pc_occ_.size() + pc_unk_.size(), 0.3);
        // if (cb_->mode_ == UAV_AutoPilot) std::cout << "boxsearch end!" << std::endl;

        if (cb_->mode_ == UAV_AutoPilot) {
            GenerateAPolytope(sfc_seed_, sfc_seed_, sfc_planes_pilot_);
            if (new_rc_flag_) AutoPilotHandle();
        } else {
            ref_path_pilot_.clear();
            ref_path_pilot_.push_back(odom_p_pilot_);
        }
        if (cb_->mode_ == UAV_Pilot) {
            if (new_rc_flag_) PilotHandle();
        }

        log_times_[1] = (ros::Time::now() - t0).toSec();

        local_pc_mutex_.unlock();

        mpc_mutex_.lock();
        cb_->user_goal_ = user_goal_pilot_;
        cb_->yaw_r_ = yaw_r_pilot_;
        sfc_planes_.resize(0, 4);
        if (sfc_planes_pilot_.rows() > 0) sfc_planes_ = sfc_planes_pilot_;
        for (int i = 0; i < ref_path_pilot_.size(); i++) {
            if (std::isnan(ref_path_pilot_[i].x()) || std::isnan(ref_path_pilot_[i].y()) || std::isnan(ref_path_pilot_[i].z())) {
                ROS_ERROR("ref_path_pilot_[i] error! nan. i: %d", i);
            }
        }
        ref_path_.clear();
        for (int i = 0; i < ref_path_pilot_.size(); i++) {
            ref_path_.push_back(ref_path_pilot_[i]);
        }
        cb_->AstarPublish(ref_path_, 2, 0.1);
        mpc_mutex_.unlock();
    }
    Eigen::Vector3d MoveToCenter(Eigen::Vector3d pt) {
        Eigen::Vector3d p;
        p.x() = (int)((pt.x() + (pt.x() > 0 ? (resolution_/2) : -(resolution_/2))) / resolution_) * resolution_;
        p.y() = (int)((pt.y() + (pt.y() > 0 ? (resolution_/2) : -(resolution_/2))) / resolution_) * resolution_;
        p.z() = (int)((pt.z() + (pt.z() > 0 ? (resolution_/2) : -(resolution_/2))) / resolution_) * resolution_;
        return p;
    }
    void GoalPublish(const ros::Publisher &pub, Eigen::Vector3d goal, double yaw) {
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = goal.x();
        msg.pose.position.y = goal.y();
        msg.pose.position.z = goal.z();
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        pub.publish(msg);
    }

    void TimerCallback(const ros::TimerEvent &);
    void HoverHandle(void);
    void PilotHandle(void);
    void AutoPilotHandle(void);
    Eigen::Vector3d SearchNewStartPoint(Eigen::Vector3d p, Eigen::Vector3d g);
    Eigen::Vector3d SearchNewEndPoint(Eigen::Vector3d p, Eigen::Vector3d g);
    bool PathReplan(Eigen::Vector3d start_p, Eigen::Vector3d end_p);
    void GenerateAPolytope(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes);

    double yaw_pilot_, yaw_r_pilot_, rc_yaw_dot_pilot_, rc_gain_pilot_;
    Eigen::Vector3d odom_p_pilot_, odom_v_pilot_, odom_a_pilot_, user_goal_pilot_, rc_joystick_pilot_, sfc_seed_;
    Eigen::Quaterniond odom_q_pilot_;

    ros::Time delay_ctrl_time_, auto_replan_time_;
    ros::Timer timer_;
    ros::Subscriber local_pc_sub_;
    ros::Publisher odom_free_pub_, goal_free_pub_;
    std::mutex timer_mutex_, local_pc_mutex_, odom_mutex_, mpc_mutex_;

    rog_map::ROGMap::Ptr map_ptr_;
    AstarClass::Ptr astar_;
    CallbackClass::Ptr cb_;
    MPCClass::Ptr mpc_;
    ciri::CIRI ciri_;

    bool esti_log_flag_, arm_flag_, replan_flag_, new_sfc_flag_, new_rc_flag_;
    int replan_cnt_;
    double esti_P_, esti_R_, resolution_;
    std::queue<std::pair<ros::Time, double>> time_thrust_;
    Eigen::Quaterniond u_q_;
    Eigen::Vector3d u_w_;

    ros::Time last_mpc_time_;
    int mpc_ctrl_index_;

    double pilot_vw_, pilot_vw_pilot_;
    Eigen::Vector3d pilot_v_, auto_pilot_v_;
    int astar_index_;
    std::vector<Eigen::Vector3d> ref_path_, ref_path_pilot_;
    vec_E<Vec3f> pc_, pc_occ_, pc_unk_;
    std::vector<double> pc_rs_;
    pcl::PointCloud<pcl::PointXYZINormal> cloud_;
    Eigen::Matrix<double, Eigen::Dynamic, 4> sfc_planes_, sfc_planes_pilot_;

    std::ofstream log_file_;
    std::vector<double> log_times_;
};

}

#endif