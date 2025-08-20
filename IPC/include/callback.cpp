#include "callback.h"

namespace ipc {

void CallbackClass::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_mutex_.lock();
    odom_flag_ = true;
    odom_time_ = msg->header.stamp;
    odom_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_v_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    odom_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    odom_a_ = odom_q_ * Eigen::Vector3d(0, 0, 1) * (thrust_ * thr2acc_) - Gravity_;
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    // std::cout << "odom: " << odom_p_.transpose() << " yaw: " << yaw_ << std::endl;
    odom_mutex_.unlock();
}

void CallbackClass::IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_mutex_.lock();
    if ((msg->header.stamp - init_time_).toSec() > 4.0 && simu_flag_ == false) {
        static int fps_count = 0;
        static ros::Time last_time;
        fps_count++;
        imu_time_ = msg->header.stamp;
        if ((imu_time_ - last_time).toSec() > 1.0) {
            last_time = imu_time_;
            if (fps_count < 100) ROS_ERROR("[IPC] Check topic: /mavros/imu/data. The frequency is %d.", fps_count);
            fps_count = 0;
        }
    }
    Eigen::Vector3d acc;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z; // body frame
    imu_gyro_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu_a_ = odom_q_ * acc;
    imu_mutex_.unlock();
}

void CallbackClass::IMURawCallback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_raw_mutex_.lock();
    if ((msg->header.stamp - init_time_).toSec() > 4.0 && simu_flag_ == false) {
        static int fps_count = 0;
        static ros::Time last_time;
        fps_count++;
        if ((msg->header.stamp - last_time).toSec() > 1.0) {
            last_time = msg->header.stamp;
            if (fps_count < 100) ROS_ERROR("[IPC] Check topic: /mavros/imu/data_raw. The frequency is %d.", fps_count);
            fps_count = 0;
        }
    }
    imu_raw_mutex_.unlock();
}

void CallbackClass::GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal_mutex_.lock();
    goal_time_ = msg->header.stamp;
    Eigen::Vector3d goal;
    goal << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    if ((goal - odom_p_).norm() > 0.1) {
        new_goal_flag_ = true;
        user_goal_ = goal;
    }
    goal_mutex_.unlock();
}

void CallbackClass::RCCallback(const mavros_msgs::RCInConstPtr& msg)
{
    rc_mutex_.lock();
    new_rc_flag_ = true;
    rc_time_ = msg->header.stamp;
    static mavros_msgs::RCIn msg_last;
    if ((rc_time_ - msg_last.header.stamp).toSec() > 1.0) {
        ROS_WARN("Receive RC again.");
        msg_last = *msg;
        rc_mutex_.unlock();
        return ;
    }

    double ch_bound = 0.1;
    rc_joystick_.x() = -(float)(msg->channels[1] - 1500) / 450.0;
    if (std::abs(rc_joystick_.x()) < ch_bound) rc_joystick_.x() = 0;
    rc_joystick_.y() = -(float)(msg->channels[0] - 1500) / 450.0;
    if (std::abs(rc_joystick_.y()) < ch_bound) rc_joystick_.y() = 0;
    rc_joystick_.z() = -(float)(msg->channels[2] - 1500) / 450.0;
    if (std::abs(rc_joystick_.z()) < 2.5*ch_bound) rc_joystick_.z() = 0;
    rc_yaw_dot_ = -(float)(msg->channels[3] - 1500) / 450.0;
    if (std::abs(rc_yaw_dot_) < ch_bound) rc_yaw_dot_ = 0;
    // std::cout << "rc_joystick: " << rc_joystick_.transpose() << " yaw_dot: " << rc_yaw_dot_ << std::endl;

    if (msg->channels[9] < 1200 && msg_last.channels[9] > 1200) rc_gain_ = 1.0;
    if (msg->channels[9] > 1800 && msg_last.channels[9] < 1800) rc_gain_ = 2.0;
    if (msg->channels[9] > 1200 && msg->channels[9] < 1800 && (msg_last.channels[9] > 1800 || msg_last.channels[9] < 1200)) rc_gain_ = 1.5;


    if (msg->channels[11] > 1800 && msg_last.channels[11] < 1800) { // reboot px4
        if (mode_ == UAV_Manual && msg->channels[10] > 1500) Px4Reboot();
        else ROS_WARN("Please turn into Manual mode to trigger PX4 reboot!");
    }
    if (msg->channels[7] > 1800 && msg_last.channels[7] < 1800) arm_flag_ = true;

    if (msg->channels[4] > 1800 && msg_last.channels[4] < 1800 && msg->channels[5] < 1500 && msg->channels[6] < 1500 && mode_ == UAV_Manual) {
        ROS_INFO("\033[1;32m[IPC FSM]: Manual --> Hover.\033[0m");
        mode_ = UAV_Hover;
        user_goal_ = init_goal_ + odom_p_;
        yaw_r_ = yaw_;
        GoalPublish(user_goal_, yaw_);
    }
    if (msg->channels[4] > 1600 && msg_last.channels[5] < 1500 && msg->channels[5] > 1500 && msg->channels[6] < 1500 && std::abs(msg->channels[2] - 1500) < 80 && mode_ == UAV_Hover) {
        ROS_INFO("\033[1;34m[IPC FSM]: Hover --> Pilot.\033[0m");
        mode_ = UAV_Pilot;
        user_goal_ = odom_p_;
    }
    // if (msg->channels[4] > 1600 && msg->channels[5] > 1500 && msg->channels[6] > 1500 && msg_last.channels[6] < 1500 && (mode_ == UAV_Pilot || mode_ == UAV_AutoPilot)) {
    //     if (mode_ == UAV_Pilot) ROS_INFO("\033[1;31m[IPC FSM]: Pilot --> Command.\033[0m");
    //     if (mode_ == UAV_AutoPilot) ROS_INFO("\033[1;31m[IPC FSM]: UAV_AutoPilot --> Command.\033[0m");
    //     mode_ = UAV_Command;
    // }
    // if (mode_ == UAV_Command) {
    //     if (msg->channels[4] < 1600) {
    //         mode_ = UAV_Manual;
    //         ROS_INFO("\033[1;33m[IPC FSM]: Command --> Manual.\033[0m");
    //     }
    //     if (msg->channels[4] > 1600 && msg->channels[5] < 1500 && msg_last.channels[5] > 1500) {
    //         mode_ = UAV_Hover;
    //         user_goal_ = odom_p_;
    //         yaw_r_ = yaw_;
    //         GoalPublish(user_goal_, yaw_r_);
    //         ROS_INFO("\033[1;32m[IPC FSM]: Command --> Hover.\033[0m");
    //     }
    //     if (msg->channels[4] > 1600 && msg->channels[5] > 1500 && msg->channels[6] < 1500 && msg_last.channels[6] > 1500) {
    //         mode_ = UAV_Pilot;
    //         user_goal_ = odom_p_;
    //         ROS_INFO("\033[1;34m[IPC FSM]: Command --> Pilot.\033[0m");
    //     }
    // }
    if (mode_ == UAV_Pilot || mode_ == UAV_AutoPilot) {
        if (mode_ != UAV_AutoPilot && msg->channels[10] > 1500 && msg_last.channels[10] < 1500) {
            mode_ = UAV_AutoPilot;
            user_goal_ = odom_p_;
            ROS_INFO("\033[1;34m[IPC FSM]: Pilot --> Auto_pilot.\033[0m");
        }
        if (msg->channels[4] < 1600) {
            mode_ = UAV_Manual;
            ROS_INFO("\033[1;33m[IPC FSM]: Pilot --> Manual.\033[0m");
        }
        if (msg->channels[4] > 1600 && msg->channels[5] < 1500 && msg_last.channels[5] > 1500) {
            mode_ = UAV_Hover;
            user_goal_ = odom_p_;
            yaw_r_ = yaw_;
            GoalPublish(user_goal_, yaw_r_);
            ROS_INFO("\033[1;32m[IPC FSM]: Pilot --> Hover.\033[0m");
        }
    }
    if (mode_ == UAV_Hover) {
        if (msg->channels[4] < 1600) {
            mode_ = UAV_Manual;
            ROS_INFO("\033[1;33m[IPC FSM]: Hover --> Manual.\033[0m");
        }
    }

    msg_last = *msg;
    rc_mutex_.unlock();
}

void CallbackClass::PVAJCmdPublish(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r)
{
    quadrotor_msgs::PositionCommand msg;
    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    msg.position.x      = p_r.x();
    msg.position.y      = p_r.y();
    msg.position.z      = p_r.z();
    msg.velocity.x      = v_r.x();
    msg.velocity.y      = v_r.y();
    msg.velocity.z      = v_r.z();
    msg.acceleration.x  = a_r.x();
    msg.acceleration.y  = a_r.y();
    msg.acceleration.z  = a_r.z();
    msg.jerk.x          = j_r.x();
    msg.jerk.y          = j_r.y();
    msg.jerk.z          = j_r.z();
    if (yaw_ctrl_flag_) {
        double yaw_error = yaw_r_ - yaw_;
        if (yaw_error >  M_PI) yaw_error -= M_PI * 2;
        if (yaw_error < -M_PI) yaw_error += M_PI * 2;
        msg.yaw     = yaw_ + yaw_error * yaw_gain_;
        msg.yaw_dot = 0;
    } else {
        msg.yaw     = 0;
        msg.yaw_dot = 0;
    }
    pvaj_cmd_pub_.publish(msg);
}

void CallbackClass::AttitudeCtrlPub(const Eigen::Quaterniond &q, const double thrust, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    if (mode_ == UAV_Manual) {
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 1;
        msg.thrust = 0.05;
    } else {
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
        msg.thrust = thrust;
    }
    if (msg.thrust < 0.5) msg.thrust = 0.05;
    if (msg.thrust > 0.9) msg.thrust = 0.9;
    if (!simu_flag_ && msg.thrust > thrust_limit_) msg.thrust = thrust_limit_;
    px4_cmd_pub_.publish(msg);
}

void CallbackClass::BodyrateCtrlPub(const Eigen::Vector3d &rate, const double thrust, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    if (mode_ == UAV_Manual) {
        msg.body_rate.x = 0;
        msg.body_rate.y = 0;
        msg.body_rate.z = 0;
        msg.thrust = 0.05;
    } else {
        msg.body_rate.x = rate.x();
        msg.body_rate.y = rate.y();
        msg.body_rate.z = rate.z();
        msg.thrust = thrust;
    }
    if (msg.thrust < 0.05) msg.thrust = 0.05;
    if (msg.thrust > 0.9) msg.thrust = 0.9;
    if (!simu_flag_ && msg.thrust > thrust_limit_) msg.thrust = thrust_limit_;
    px4_cmd_pub_.publish(msg);
}

void CallbackClass::GoalPublish(Eigen::Vector3d goal, double yaw)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = goal.x();
    msg.pose.position.y = goal.y();
    msg.pose.position.z = goal.z();
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    goal_pub_.publish(msg);
}

void CallbackClass::AstarPublish(std::vector<Eigen::Vector3d>& nodes, uint8_t type, double scale) {
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    if (type == 0) {
        node_vis.ns = "astar";
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    } else if (type == 1) {
        node_vis.ns = "floyd";
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    } else if (type == 2) {
        node_vis.ns = "short";
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 1.0;
    } else if (type == 3) {
        node_vis.ns = "ref";
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;
    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    
    node_vis.scale.x = scale;
    node_vis.scale.y = scale;
    node_vis.scale.z = scale;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        node_vis.points.push_back(pt);
    }
    astar_pub_.publish(node_vis);
}

void CallbackClass::GridMapPublish(pcl::PointCloud<pcl::PointXYZ>& pc) {
    pc.width    = pc.points.size();
    pc.height   = 1;
    pc.is_dense = true;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pc, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    ogm_pub_.publish(msg);
}

void CallbackClass::MPCPathPublish(std::vector<Eigen::Vector3d> &pt) {
    nav_msgs::Path msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    for (int i = 0; i < pt.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt[i].x();
        pose.pose.position.y = pt[i].y();
        pose.pose.position.z = pt[i].z();
        // std::cout << i << " " << pt[i].transpose() << std::endl;
        msg.poses.push_back(pose);
    }
    mpc_path_pub_.publish(msg);
}

}