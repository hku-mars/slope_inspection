//
// Created by yunfan on 2022/3/26.
//

#pragma once

#include "ros/ros.h"
#include "vector"
#include "string"
#include "config.hpp"
#include "nav_msgs/Path.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/RCIn.h"
#include "Eigen/Core"
#include "traj_opt/waypt_traj_opt.hpp"
#include "geometry_utils/geometry_utils.h"
#include "geometry_utils/polynomial_interpolation.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/MpcPositionCommand.h"
#include "visualization_utils/visualization_utils.h"
#include "geometry_utils/trajectory.h"

namespace waypoint_planner {
    using namespace geometry_utils;
    using namespace type_utils;
    using namespace visualization_utils;
    using std::cout;
    using std::endl;
    using namespace traj_opt;

    class WaypointPlanner {
    private:
        MissionConfig cfg_;

        ros::NodeHandle nh_;

        Eigen::Vector3d cur_position, cur_vel;
        double cur_yaw;
        int waypoint_counter{0};
        bool had_odom{false};
        bool plan_success{false};
        bool triggered{false};
        bool new_goal{true};
        double traj_start_WT;
        double odom_rcv_time{0};
        ros::Publisher goal_pub_, path_pub_, mkr_pub_, pid_cmd_pub_, mpc_cmd_pub_;
        ros::Subscriber click_sub_, mavros_sub_, odom_sub_;
        ros::Timer goal_pub_timer_;

        WayptTrajOpt::Ptr traj_opt_;

        Trajectory pos_traj, yaw_traj;


        void OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
            had_odom = true;
            odom_rcv_time = ros::Time::now().toSec();
            cur_position = Eigen::Vector3d(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
            Quatf q(msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z);
            cur_yaw = geometry_utils::get_yaw_from_quaternion(q);
            cur_vel = Eigen::Vector3d(msg->twist.twist.linear.x,
                                      msg->twist.twist.linear.y,
                                      msg->twist.twist.linear.z);
        }

        bool CloseToPoint(Vec3f &position) {
            return (position - cur_position).norm() < cfg_.switch_dis;
        }

        void RvizClickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            if (!had_odom) {
                cout << RED " -- [MISSION] No odom received." << RESET << endl;
                return;
            }
            triggered = true;
            if (GetPosTrajPlanning()) {
                plan_success = true;
            } else {
                plan_success = false;
                triggered = false;
            }
            new_goal = true;
            waypoint_counter = 0;
            cout << YELLOW << " -- [MISSION] Rviz triggered." << RESET << endl;
        }

        void MavrosRcCallback(const mavros_msgs::RCInConstPtr &msg) {
            static int last_ch_10 = 1000;
            if (!had_odom) {
                return;
            }
            int ch_10 = msg->channels[9];
            bool pitch_up = msg->channels[1] < 1200;
            if (last_ch_10 > 1500 && ch_10 < 1500) {
                triggered = true;
                if (GetPosTrajPlanning()) {
                    plan_success = true;

                } else {
                    plan_success = false;
                    triggered = false;
                }
                new_goal = true;
                waypoint_counter = 0;
                cout << YELLOW << " -- [MISSION] Mavros triggered." << RESET << endl;
            }
            last_ch_10 = ch_10;
        }

        void CmdPubTimerCallback(const ros::TimerEvent &e) {

            if (!triggered || !had_odom || !plan_success) {
                return;
            }

            double eval_t = (ros::Time::now().toSec() - traj_start_WT);
            eval_t = eval_t > pos_traj.getTotalDuration() ? pos_traj.getTotalDuration() : eval_t;
            cout << GREEN << " -- [MISSION] eval_t: " << eval_t << RESET << endl;
            double yaw, yaw_dot;


            // pub for pid
            {
                quadrotor_msgs::PositionCommand cmds;
                cmds.header.frame_id = "world";
                cmds.header.stamp = ros::Time::now();

                double dt = eval_t;
                if (dt >= pos_traj.getTotalDuration())
                    dt = pos_traj.getTotalDuration();
                Vec3f pos = pos_traj.getPos(dt);
                Vec3f vel = pos_traj.getVel(dt);
                Vec3f acc = pos_traj.getAcc(dt);
                Vec3f jer = pos_traj.getJer(dt);
                if (!yaw_traj.empty()) {
                    yaw = yaw_traj.getPos(dt)(0);
                    yaw_dot = yaw_traj.getVel(dt)(0);
                }
                if (dt >= pos_traj.getTotalDuration() - 0.01) {
                    jer = Vec3f(0, 0, 0);
                } else {
                    jer = pos_traj.getJer(dt);
                }

                cmds.position.x = pos.x();
                cmds.position.y = pos.y();
                cmds.position.z = pos.z();
                cmds.velocity.x = vel.x();
                cmds.velocity.y = vel.y();
                cmds.velocity.z = vel.z();
                cmds.acceleration.x = acc.x();
                cmds.acceleration.y = acc.y();
                cmds.acceleration.z = acc.z();
                cmds.jerk.x = jer.x();
                cmds.jerk.y = jer.y();
                cmds.jerk.z = jer.z();
                cmds.yaw = yaw;
                cmds.yaw_dot = yaw_dot;
//                if (pp_.use_yaw == false) {
//                    cmds.yaw = 0.0;
//                    cmds.yaw_dot = 0.0;
//                } else {
//                    std::pair<double, double> yyd = calculate_yaw(traj_to_pub, dt, pos, pp_.dt);
//                    cmds.yaw = yyd.first;
//                    cmds.yaw_dot = yyd.second;
//                }

                pid_cmd_pub_.publish(cmds);
            }

            //发布飞向起点的轨迹
            // && abs(automode - 1) < 0.1
            // for mpc
            {
                quadrotor_msgs::MpcPositionCommand cmds;
                cmds.cmds.resize(8);

                cmds.header.frame_id = "world";
                cmds.header.stamp = ros::Time::now();
                cmds.mpc_horizon = 8;
                for (size_t i = 0; i < 8; i++) {
                    double dt = 0.033 * i + eval_t;
                    if (dt >= pos_traj.getTotalDuration())
                        dt = pos_traj.getTotalDuration();
                    if (!yaw_traj.empty()) {
                        yaw = yaw_traj.getPos(dt)(0);
                        yaw_dot = yaw_traj.getVel(dt)(0);
                    }
                    Vec3f pos = pos_traj.getPos(dt);
                    Vec3f vel = pos_traj.getVel(dt);
                    Vec3f acc = pos_traj.getAcc(dt);
                    Vec3f jer = pos_traj.getJer(dt);
                    if (dt >= pos_traj.getTotalDuration() - 0.01) {
                        jer = Vec3f(0, 0, 0);
                    } else {
                        jer = pos_traj.getJer(dt);
                    }
                    cmds.cmds[i].position.x = pos.x();
                    cmds.cmds[i].position.y = pos.y();
                    cmds.cmds[i].position.z = pos.z();
                    cmds.cmds[i].velocity.x = vel.x();
                    cmds.cmds[i].velocity.y = vel.y();
                    cmds.cmds[i].velocity.z = vel.z();
                    cmds.cmds[i].acceleration.x = acc.x();
                    cmds.cmds[i].acceleration.y = acc.y();
                    cmds.cmds[i].acceleration.z = acc.z();
                    cmds.cmds[i].jerk.x = jer.x();
                    cmds.cmds[i].jerk.y = jer.y();
                    cmds.cmds[i].jerk.z = jer.z();
                    cmds.cmds[i].yaw = yaw;
                    cmds.cmds[i].yaw_dot = yaw_dot;
//                    if (pp_.use_yaw == false) {
//                        cmds.cmds[i].yaw = 0.0;
//                        cmds.cmds[i].yaw_dot = 0.0;
//                    } else {
//                        std::pair<double, double> yyd = calculate_yaw(traj_to_pub, dt, pos, pp_.dt);
//                        cmds.cmds[i].yaw = yyd.first;
//                        cmds.cmds[i].yaw_dot = yyd.second;
//                    }
                }
                mpc_cmd_pub_.publish(cmds);
            }

        }


        /// The Goal for yaw planning is find the yaw waypoint close to the pos traj and plan
        /// the minsnap yaw traj.
        bool GetYawPlanning(const Trajectory &pos_traj) {
            if (cfg_.yaw_dot_max < 1e-3)
                return false;
            Matrix<double, 1, 4> goal_state(cfg_.yaw_waypoints(cfg_.yaw_waypoints.size() - 1), 0, 0, 0);
            Matrix<double, 1, 4> init_state(cur_yaw, 0, 0, 0);
            Matrix<double, 1, -1> way_pts;
            VecDf times = pos_traj.getDurations();

            VecDf yaw_times(times.size() / 2);
            for (int i = 0; i < yaw_times.size(); ++i) {
                yaw_times(i) = times(2 * i) + times(2 * i + 1);
            }
            cout << "Yaw planning" << endl;
            cout << init_state.transpose() << endl;
            cout << "====================" << endl;
            cout << yaw_times.transpose() << endl;
            cout << "====================" << endl;
            cout << goal_state.transpose() << endl;
            Eigen::Matrix<double, 1, -1> waypts = cfg_.yaw_waypoints.head(cfg_.yaw_waypoints.size() - 1);
            yaw_traj = poly_interpo::minimumSnapInterpolation<1>(init_state,
                                                                 goal_state,
                                                                 waypts,
                                                                 yaw_times);
            yaw_traj.start_WT = pos_traj.start_WT;
            return true;

        }

        bool GetPosTrajPlanning() {
            StatePVAJ init_state, goal_state;
            init_state.setZero();
            goal_state.setZero();
            init_state.col(0) = cur_position;
            if (cur_vel.norm() > 0.3) {
                init_state.col(1) = cur_vel;
            }
            MatDf inner_pts = cfg_.waypoints.leftCols(cfg_.waypoints.cols() - 1);
            if ((cur_position - cfg_.waypoints.col(0)).norm() < 0.1) {
                inner_pts = inner_pts.rightCols(inner_pts.cols() - 1);
            }
            goal_state.col(0) = cfg_.waypoints.rightCols(1);
            cout << GREEN " -- [MISSION] Begin traj_opt." << RESET << endl;
            cout << init_state << endl;
            cout << "====================" << endl;
            cout << cfg_.waypoints.leftCols(cfg_.waypoints.cols() - 1) << endl;
            cout << "====================" << endl;
            cout << goal_state << endl;
            cout << "====================" << endl;
            if (traj_opt_->optimize(init_state, goal_state, inner_pts, pos_traj)) {
                VisualUtils::DeleteMkrArr(mkr_pub_);
                VisualUtils::VisualizeTrajectoryInColorVel(mkr_pub_, pos_traj, 0.03, "color_traj", 0.05);
                if (cfg_.yaw_waypoints.size() != 0) {
                    if (!GetYawPlanning(pos_traj)) {
                        return false;
                    }
                }
                traj_start_WT = ros::Time::now().toSec();
                return true;
            }
            return false;
        }

    public:
        WaypointPlanner() {};

        WaypointPlanner(
                const ros::NodeHandle &nh) {
            nh_ = nh;
            cfg_ = MissionConfig(nh_);
            traj_opt_.reset(new WayptTrajOpt(nh_, cfg_.opt_cfg));
            odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &WaypointPlanner::OdomCallback, this);
            goal_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &WaypointPlanner::CmdPubTimerCallback, this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_pub_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.path_pub_topic, 10);
            click_sub_ = nh_.subscribe("/goal", 10, &WaypointPlanner::RvizClickCallback, this);
            mavros_sub_ = nh_.subscribe("/mavros/rc/in", 10, &WaypointPlanner::MavrosRcCallback, this);
            mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualize", 10);
            pid_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
            mpc_cmd_pub_ = nh_.advertise<quadrotor_msgs::MpcPositionCommand>("/planning_cmd/mpc", 10);
        }
    };
}
