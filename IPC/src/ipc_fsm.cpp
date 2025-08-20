#include "ipc_fsm.h"

#define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward{
//     backward::SignalHandling sh;
// }

namespace ipc {

void IPCFSMClass::TimerCallback(const ros::TimerEvent &)
{
    timer_mutex_.lock();
    if (cb_->mode_ == UAV_Manual || cb_->odom_flag_ == false) { // initial state, return
        if (cb_->uav_state_.mode == "OFFBOARD") {
            static int count = 0;
            count ++;
            if (count * (1.0 / cb_->freq_) > 0.1) {
                count = 0;
                // cb_->SetManualMode();
            }
        }
        if (cb_->odom_flag_ == false) {
            static ros::Time log_time;
            if ((ros::Time::now() - log_time).toSec() > 2.0) {
                ROS_ERROR("[IPC] No odom!");
                log_time = ros::Time::now();
            }
        }
        arm_flag_ = false;
        u_q_.setIdentity();
        u_w_.setZero();
        cb_->BodyrateCtrlPub(Eigen::Vector3d(0,0,0), 0.05, ros::Time::now());
        timer_mutex_.unlock();
        return;
    }

    ros::Time t0 = ros::Time::now();
    if (cb_->uav_state_.mode != "OFFBOARD") {
        static int count = 0;
        count ++;
        if (count * (1.0 / cb_->freq_) > 0.1) {
            count = 0;
            // cb_->SetOffboardMode();
        }
    }

    static UAV_mode_e mode_last;
    if (mode_last != cb_->mode_) {
        mode_last = cb_->mode_;
        astar_index_ = 0;
        ref_path_.clear();
    }
    bool add_sfc_flag = true;
    // bool add_sfc_flag = false;
    switch (cb_->mode_) {
        case UAV_Hover:
            HoverHandle();
            break;
        case UAV_Pilot:
            // PilotHandle();
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetGoal(cb_->user_goal_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
            break;
        case UAV_AutoPilot:
            mpc_mutex_.lock();
            if (sfc_planes_.rows() == 0) {
                add_sfc_flag = false;
                ROS_WARN("[fsm]: No sfc!");
            } else {
                if (mpc_->IsInFSC(cb_->odom_p_, sfc_planes_) == false) {
                    add_sfc_flag = false;
                    ROS_WARN("[fsm]: Cur odom is not in sfc!");
                }
            }
            if (ref_path_.size() > 0) {
                double min_dis = 10000.0;
                for (int i = 0; i < ref_path_.size(); i++) { // find the nearest point
                    double dis = (cb_->odom_p_ - ref_path_[i]).norm();
                    if (dis < min_dis) {
                        min_dis = dis;
                        astar_index_ = i;
                    }
                }
                std::vector<Eigen::Vector3d> mpc_goals;
                Eigen::Vector3d last_p_ref;
                for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                    if (add_sfc_flag) mpc_->SetFSC(sfc_planes_, i);
                    int index = astar_index_ + i * cb_->vel_ref_ * mpc_->MPC_STEP / cb_->path_dis_;
                    if (index >= ref_path_.size()) index = ref_path_.size() - 1;
                    assert(index >= 0);
                    Eigen::Vector3d v_r(0, 0, 0);
                    if (i == 0) v_r = (ref_path_[index] - cb_->odom_p_) / mpc_->MPC_STEP;
                    else if (i == mpc_->MPC_HORIZON) v_r.setZero();
                    else v_r = (ref_path_[index] - last_p_ref) / mpc_->MPC_STEP;
                    last_p_ref = ref_path_[index];
                    mpc_->SetGoal(ref_path_[index], v_r, Eigen::Vector3d::Zero(), i);
                    mpc_goals.push_back(ref_path_[index]);
                }
                cb_->AstarPublish(mpc_goals, 3, astar_->resolution_);
            } else {
                for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                    if (add_sfc_flag) mpc_->SetFSC(sfc_planes_, i);
                    mpc_->SetGoal(cb_->user_goal_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
                }
                // ROS_ERROR("no path! hover! pt: %f, %f, %f", user_goal_pilot_.x(), user_goal_pilot_.y(), user_goal_pilot_.z());
            }
            mpc_mutex_.unlock();
            break;
        default:
            break;
    }
    log_times_[0] = (ros::Time::now() - t0).toSec();

    // backend step 2: model predictive control(MPC)
    ros::Time t0_mpc = ros::Time::now();
    mpc_->SetStatus(cb_->odom_p_, cb_->odom_v_, cb_->odom_a_);
    bool success_flag = mpc_->Run();
    Eigen::Vector3d p_opt, v_opt, a_opt, u_opt, u_pret;
    Eigen::MatrixXd A1, B1;
    Eigen::VectorXd x_opt = mpc_->X_0_;
    if (success_flag) { // todo: use odom time to calculate current angular velocity
        last_mpc_time_ = t0_mpc;
        mpc_ctrl_index_ = cb_->ctrl_delay_ / mpc_->MPC_STEP;
        for (int i = 0; i <= cb_->ctrl_delay_ / mpc_->MPC_STEP; i++) {
            mpc_->GetOptimCmd(u_opt, i);
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
            x_opt = A1 * x_opt + B1 * u_opt;
        }
        p_opt << x_opt(0,0), x_opt(1,0), x_opt(2,0);
        v_opt << x_opt(3,0), x_opt(4,0), x_opt(5,0);
        a_opt << x_opt(6,0), x_opt(7,0), x_opt(8,0);
        cb_->PVAJCmdPublish(cb_->odom_p_, v_opt, a_opt, u_opt);

        std::vector<Eigen::Vector3d> path;
        x_opt = mpc_->X_0_;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->GetOptimCmd(u_pret, i);
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
            x_opt = A1 * x_opt + B1 * u_pret;
            path.push_back(Eigen::Vector3d(x_opt(0,0), x_opt(1,0), x_opt(2,0)));
        }
        cb_->MPCPathPublish(path);
    } else { // debug
        double dt = (ros::Time::now() - last_mpc_time_).toSec();
        int index = mpc_ctrl_index_;
        if (dt >= mpc_->MPC_STEP) {
            index += dt / mpc_->MPC_STEP;
            last_mpc_time_ = ros::Time::now();
        }
        mpc_->GetOptimCmd(u_opt, index);
        mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
        x_opt = A1 * mpc_->X_0_ + B1 * u_opt;
        p_opt << x_opt(0,0), x_opt(1,0), x_opt(2,0);
        v_opt << x_opt(3,0), x_opt(4,0), x_opt(5,0);
        a_opt << x_opt(6,0), x_opt(7,0), x_opt(8,0);
        cb_->PVAJCmdPublish(cb_->odom_p_, v_opt, a_opt, u_opt);
    }
    log_times_[4] = (ros::Time::now() - t0_mpc).toSec();

    // backend step 3: differential flatness (DF)
    ros::Time t0_df = ros::Time::now();
    estimateThrustModel(cb_->imu_a_);
    a_opt = a_opt + cb_->Gravity_;
    ComputeThrust(a_opt);
    ConvertCommand(a_opt, u_opt);
    if (cb_->bodyrate_flag_) cb_->BodyrateCtrlPub(u_w_, cb_->thrust_, t0_df);
    else cb_->AttitudeCtrlPub(u_q_, cb_->thrust_, t0_df);
    time_thrust_.push(std::pair<ros::Time, double>(t0_df, cb_->thrust_));
    while (time_thrust_.size() > 100) {time_thrust_.pop();}
    log_times_[5] = (ros::Time::now() - t0_df).toSec();
    log_times_[6] = (ros::Time::now() - t0).toSec();
    TimeLogWrite();

    timer_mutex_.unlock();
}

void IPCFSMClass::HoverHandle(void)
{
    static ros::Time arm_time;
    if (cb_->arm_flag_ == true) {
        cb_->arm_flag_ = false;
        arm_time = ros::Time::now();
    }
    if ((ros::Time::now() - arm_time).toSec() < 1.0) {
        cb_->SwitchArmedCmd(true);
    }
    for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
        mpc_->SetGoal(cb_->user_goal_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
    }
}

// void IPCFSMClass::PilotHandle(void)
// {
//     Eigen::Vector3d rc_joystick_w;
//     if (cb_->new_rc_flag_) {
//         cb_->new_rc_flag_ = false;
//         double pred_t = 1.0; // user preference (1s)
//         // Eigen::Vector3d rc_joystick_w;
//         rc_joystick_w.x() = cb_->rc_gain_ * (cb_->rc_joystick_.x() * std::cos(cb_->yaw_) - cb_->rc_joystick_.y() * std::sin(cb_->yaw_));
//         rc_joystick_w.y() = cb_->rc_gain_ * (cb_->rc_joystick_.y() * std::cos(cb_->yaw_) + cb_->rc_joystick_.x() * std::sin(cb_->yaw_));
//         rc_joystick_w.z() = cb_->rc_gain_ * cb_->rc_joystick_.z();
//         if (std::fabs(rc_joystick_w.x()) > cb_->rc_gain_ * 0.1) cb_->user_goal_.x() = cb_->odom_p_.x() + rc_joystick_w.x() * pilot_v_.x() * pred_t;
//         if (std::fabs(rc_joystick_w.y()) > cb_->rc_gain_ * 0.1) cb_->user_goal_.y() = cb_->odom_p_.y() + rc_joystick_w.y() * pilot_v_.y() * pred_t;
//         if (std::fabs(rc_joystick_w.z()) > cb_->rc_gain_ * 0.1) cb_->user_goal_.z() = cb_->odom_p_.z() + rc_joystick_w.z() * pilot_v_.z() * pred_t;
//         if (std::fabs(cb_->rc_yaw_dot_)  > 0.1) cb_->yaw_r_ = cb_->yaw_ + pilot_vw_ * cb_->rc_yaw_dot_ * pred_t;
//         if (cb_->yaw_r_ >  M_PI) cb_->yaw_r_ -= 2 * M_PI;
//         if (cb_->yaw_r_ < -M_PI) cb_->yaw_r_ += 2 * M_PI;
//         cb_->GoalPublish(cb_->user_goal_, cb_->yaw_r_);
//     }
//     for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
//         mpc_->SetGoal(cb_->user_goal_, rc_joystick_w, Eigen::Vector3d::Zero(), i);
//     }
// }

void IPCFSMClass::PilotHandle(void)
{
    double pred_t = 1.0; // user preference (1s)
    Eigen::Vector3d rc_joystick_w;
    rc_joystick_w.x() = rc_gain_pilot_ * (rc_joystick_pilot_.x() * std::cos(yaw_pilot_) - rc_joystick_pilot_.y() * std::sin(yaw_pilot_));
    rc_joystick_w.y() = rc_gain_pilot_ * (rc_joystick_pilot_.y() * std::cos(yaw_pilot_) + rc_joystick_pilot_.x() * std::sin(yaw_pilot_));
    rc_joystick_w.z() = rc_gain_pilot_ * rc_joystick_pilot_.z();
    if (std::fabs(rc_joystick_w.x()) > rc_gain_pilot_ * 0.1) user_goal_pilot_.x() = odom_p_pilot_.x() + rc_joystick_w.x() * pilot_v_.x() * pred_t;
    if (std::fabs(rc_joystick_w.y()) > rc_gain_pilot_ * 0.1) user_goal_pilot_.y() = odom_p_pilot_.y() + rc_joystick_w.y() * pilot_v_.y() * pred_t;
    if (std::fabs(rc_joystick_w.z()) > rc_gain_pilot_ * 0.1) user_goal_pilot_.z() = odom_p_pilot_.z() + rc_joystick_w.z() * pilot_v_.z() * pred_t;
    if (std::fabs(rc_yaw_dot_pilot_)  > 0.1) yaw_r_pilot_ = yaw_pilot_ + pilot_vw_pilot_ * rc_yaw_dot_pilot_ * pred_t;
    if (yaw_r_pilot_ >  M_PI) yaw_r_pilot_ -= 2 * M_PI;
    if (yaw_r_pilot_ < -M_PI) yaw_r_pilot_ += 2 * M_PI;
    cb_->GoalPublish(user_goal_pilot_, yaw_r_pilot_);
}

void IPCFSMClass::AutoPilotHandle(void)
{
    double pred_t = 1.0; // user preference (1s)
    Eigen::Vector3d rc_joystick_w(0, 0, 0), rc_goal = user_goal_pilot_;
    rc_joystick_w.x() = rc_gain_pilot_ * (rc_joystick_pilot_.x() * std::cos(yaw_pilot_) - rc_joystick_pilot_.y() * std::sin(yaw_pilot_));
    rc_joystick_w.y() = rc_gain_pilot_ * (rc_joystick_pilot_.y() * std::cos(yaw_pilot_) + rc_joystick_pilot_.x() * std::sin(yaw_pilot_));
    rc_joystick_w.z() = rc_gain_pilot_ * rc_joystick_pilot_.z();
    for (int i = 0; i < 3; i++) {
        if (std::fabs(rc_joystick_w(i)) > rc_gain_pilot_ * 0.1) {
            rc_goal(i) = odom_p_pilot_(i) + rc_joystick_w(i) * auto_pilot_v_(i) * pred_t;
        }
    }
    rc_goal = MoveToCenter(rc_goal);
    if (std::fabs(rc_yaw_dot_pilot_) > 0.1) yaw_r_pilot_ = yaw_pilot_ + pilot_vw_pilot_ * rc_yaw_dot_pilot_ * pred_t;
    if (yaw_r_pilot_ >  M_PI) yaw_r_pilot_ -= 2 * M_PI;
    if (yaw_r_pilot_ < -M_PI) yaw_r_pilot_ += 2 * M_PI;
    cb_->GoalPublish(rc_goal, yaw_r_pilot_); // user predictive goal
    
    bool search_odom_flag = false, search_goal_flag = false, search_goal_success_flag = false;
    Eigen::Vector3d Grid_odom_free = odom_p_pilot_, Grid_goal_free = rc_goal;

    // handle cur odom
    if (map_ptr_->isOccupiedInflate(odom_p_pilot_) || map_ptr_->isUnknownInflate(odom_p_pilot_)) {
        search_odom_flag = true;
        Eigen::Vector3d p0;
        p0 = MoveToCenter(odom_p_pilot_);
        bool find_free_flag = false;
        for (int z = 0; z <= 5; z++) {
            for (int dir = 1; dir >= -1; dir -= 2) {
                for (int i = 1; i < 0.4/resolution_; i++) { // search radius must large
                    for (int x = -1; x <= 1; x++) {
                        for (int y = -1; y <= 1; y++) {
                            if (x == 0 && y == 0 && z == 0) continue;
                            Eigen::Vector3d pt;
                            pt.x() = p0.x() + x * resolution_ * i;
                            pt.y() = p0.y() + y * resolution_ * i;
                            pt.z() = p0.z() + z * resolution_ * dir;
                            if (map_ptr_->isFreeInflate(pt)) {
                                if (map_ptr_->isOccupiedInflate(pt)) ROS_ERROR("rog error! new free odom is free and occ in inf map!");
                                if (map_ptr_->isUnknownInflate(pt)) ROS_ERROR("rog error! new free odom is free and unk in inf map!");
                                Grid_odom_free = pt;
                                find_free_flag = true;
                                break;
                            }
                        }
                        if (find_free_flag) break;
                    }
                    if (find_free_flag) break;
                }
                if (z == 0) continue;
                if (find_free_flag) break;
            }
            if (find_free_flag) break;
        }
        if (find_free_flag) ROS_WARN("Find a grid free near odom! %f %f %f", Grid_odom_free.x(), Grid_odom_free.y(), Grid_odom_free.z());
        else ROS_ERROR("Can not find a grid free near odom!");
    }
    if (search_odom_flag) GoalPublish(odom_free_pub_, Grid_odom_free, yaw_r_pilot_);


    // handle goal
    if (map_ptr_->isOccupiedInflate(rc_goal) || map_ptr_->isUnknownInflate(rc_goal)) {
        search_goal_flag = true;
        bool find_free_flag = false;
        double search_r = std::max(0.4, (odom_p_pilot_ - rc_goal).norm());
        int num = search_r / resolution_;
        for (int z = 0; z <= 5; z++) {
            for (int dir = 1; dir >= -1; dir -= 2) {
                for (int i = 1; i < num; i++) {
                    double dis_min = 10000.0;
                    for (int x = -1; x <= 1; x++) {
                        for (int y = -1; y <= 1; y++) {
                            if (x == 0 && y == 0 && z == 0) continue;
                            Eigen::Vector3d pt;
                            pt.x() = rc_goal.x() + x * resolution_ * i;
                            pt.y() = rc_goal.y() + y * resolution_ * i;
                            pt.z() = rc_goal.z() + z * resolution_ * dir;
                            pt = MoveToCenter(pt);
                            if (map_ptr_->isFreeInflate(pt)) {
                                if (map_ptr_->isOccupiedInflate(pt)) ROS_ERROR("rog error! new goal is free and occ in inf map!");
                                if (map_ptr_->isUnknownInflate(pt)) ROS_ERROR("rog error! new goal is free and unk in inf map!");
                                bool free_flag = true;
                                Eigen::Vector3d raycast_vec = pt - Grid_odom_free;
                                int raycast_num = raycast_vec.norm() / resolution_;
                                if (raycast_num > 0) {
                                    for (int k = 1; k < raycast_num; k++) {
                                        Eigen::Vector3d raycast_pt = Grid_odom_free + raycast_vec * k / raycast_num;
                                        if (map_ptr_->isFreeInflate(raycast_pt) == false) {
                                            // if (map_ptr_->isOccupiedInflate(pt)) ROS_ERROR("rog error! ray goal 1 is free and occ in inf map!");
                                            // if (map_ptr_->isUnknownInflate(pt)) ROS_ERROR("rog error! ray goal 1  is free and unk in inf map!");
                                            free_flag = false;
                                            break;
                                        }
                                    }
                                }
                                if (free_flag == false) continue;
                                // double dis = (pt - Grid_odom_free).norm();
                                double dis = std::fabs((pt - Grid_odom_free).norm() - raycast_vec.norm());
                                if (dis < dis_min) {
                                    dis_min = dis;
                                    Grid_goal_free = pt;
                                }
                                search_goal_success_flag = true;
                                find_free_flag = true;
                            }
                        }
                    }
                    if (find_free_flag) break;
                }
                if (z == 0) continue;
                if (find_free_flag) break;
            }
            if (find_free_flag) {
                ROS_INFO("find a new goal free near rc_goal");
                break;
            }
        }
    } else {
        Eigen::Vector3d og_vec = Grid_goal_free - Grid_odom_free;
        int og_num = og_vec.norm() / resolution_ * 2;
        if (og_num > 0) {
            for (int i = 1; i <= og_num; i++) {
                Eigen::Vector3d pt = Grid_odom_free + og_vec * i / og_num;
                pt = MoveToCenter(pt);
                if (map_ptr_->isFreeInflate(pt) == false) {
                    break;
                }
                Grid_goal_free = pt;
                search_goal_success_flag = true;
            }
        } else {
            // ROS_WARN("Goal free. But reject this goal!");
        }
    }
    if (search_goal_flag) GoalPublish(goal_free_pub_, Grid_goal_free, yaw_r_pilot_);
    

    // path generation
    if (search_goal_success_flag) {
        Eigen::Vector3d vector = Grid_goal_free - Grid_odom_free;
        int num = vector.norm() / resolution_;
        if (num > 0) {
            ref_path_pilot_.clear();
            for (int i = 0; i <= num; i++) {
                Eigen::Vector3d pt = Grid_odom_free + vector * i / num;
                ref_path_pilot_.push_back(pt);
            }
            user_goal_pilot_ = Grid_goal_free;
        } else {
            ref_path_pilot_.clear();
            Grid_odom_free = MoveToCenter(Grid_odom_free);
            ref_path_pilot_.push_back(Grid_odom_free);
            // ref_path_pilot_.push_back(Grid_goal_free);
            // ROS_WARN("Goal and odom too near. Reject this goal!");
        }
        // for (int i = 0; i < ref_path_pilot_.size(); i++) {
        //     if (std::isnan(ref_path_pilot_[i].x()) || std::isnan(ref_path_pilot_[i].y()) || std::isnan(ref_path_pilot_[i].z())) {
        //         ROS_ERROR("local_pc: ref_path_pilot_[%d] error! nan.", i);
        //     }
        // }
    }

    // if (search_goal_flag) {
    //     if (search_goal_success_flag) {
    //         Eigen::Vector3d vector = Grid_goal_free - Grid_odom_free;
    //         int num = vector.norm() / resolution_;
    //         if (num > 0) {
    //             ref_path_pilot_.clear();
    //             for (int i = 0; i <= num; i++) {
    //                 Eigen::Vector3d pt = Grid_odom_free + vector * i / num;
    //                 ref_path_pilot_.push_back(pt);
    //             }
    //             user_goal_pilot_ = Grid_goal_free;
    //         }
    //         if (std::isnan(ref_path_pilot_[0].x()) || std::isnan(ref_path_pilot_[0].y()) || std::isnan(ref_path_pilot_[0].z())) {
    //             ROS_ERROR("ref_path_pilot_[0] error! nan.");
    //         }
    //     } else {
    //         // std::cout << "Can not search free goal. use old ref_path_pilot_." << std::endl;
    //     }
    // } else {
    //     Eigen::Vector3d vector = rc_goal - Grid_odom_free;
    //     int num = vector.norm() / resolution_ * 2;
    //     for (int i = 1; i <= num; i++) {
    //         Eigen::Vector3d pt = Grid_odom_free + vector * i / num;
    //         if (map_ptr_->isFreeInflate(pt) == false) break;
    //         rc_goal = pt;
    //     }
    //     vector = rc_goal - Grid_odom_free;
    //     num = vector.norm() / resolution_;
    //     if (num > 0) { // to near will fail!!!
    //         ref_path_pilot_.clear();
    //         for (int i = 0; i <= num; i++) {
    //             Eigen::Vector3d pt = Grid_odom_free + vector * i / num;
    //             ref_path_pilot_.push_back(pt);
    //         }
    //         user_goal_pilot_ = rc_goal;
    //     } else {
    //         // std::cout << "Odom and goal are free, but raycast fails. Use old ref_path_pilot_." << std::endl;
    //         // ROS_ERROR("Odom and goal are free, but raycast fails. Use old ref_path_pilot_.");
    //     }
    // }
    
}

Eigen::Vector3d IPCFSMClass::SearchNewStartPoint(Eigen::Vector3d p, Eigen::Vector3d g)
{
    Eigen::Vector3d new_p = p;
    if (map_ptr_->isOccupiedInflate(p)) {
        int point_num = 12;
        double r = astar_->resolution_;
        while (true) {
            bool flag = false;
            double dis_min = 10000.0;
            for (int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << p.x() + r*std::sin(M_PI*2*i/point_num), p.y() + r*std::cos(M_PI*2*i/point_num), p.z();
                double dis = (g - pt).norm(); 
                if (map_ptr_->isOccupiedInflate(pt) == false) {
                    if (dis < dis_min) {
                        dis_min = dis;
                        new_p = pt;
                        flag = true;
                    }
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change start point from %f %f %f to %f %f %f.\033[0m", p.x(), p.y(), p.z(), new_p.x(), new_p.y(), new_p.z());
                break;
            }
            r += astar_->resolution_;
        }
    }
    return new_p;
}

Eigen::Vector3d IPCFSMClass::SearchNewEndPoint(Eigen::Vector3d p, Eigen::Vector3d g)
{
    Eigen::Vector3d new_p = p;
    if (map_ptr_->isOccupiedInflate(p)) {
        int point_num = 12;
        double r = astar_->resolution_;
        while (true) {
            bool flag = false;
            double dis_max = -1.0;
            for (int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << p.x() + r*std::sin(M_PI*2*i/point_num), p.y() + r*std::cos(M_PI*2*i/point_num), p.z();
                double dis = (g - pt).norm(); 
                if (map_ptr_->isOccupiedInflate(pt) == false) {
                    if (dis > dis_max) {
                        dis_max = dis;
                        new_p = pt;
                        flag = true;
                    }
                }
            }
            if (flag) {
                ROS_WARN("Change end point from %f %f %f to %f %f %f.", p.x(), p.y(), p.z(), new_p.x(), new_p.y(), new_p.z());
                break;
            }
            r += astar_->resolution_;
        }
    }
    return new_p;
}

bool IPCFSMClass::PathReplan(Eigen::Vector3d start_p, Eigen::Vector3d end_p)
{
    // ros::Time t0 = ros::Time::now();
    // ref_path2_.clear();
    // std::vector<Eigen::Vector3d> astar_path, waypoints;

    // start_p = SearchNewStartPoint(start_p, odom_p_);
    // end_p = SearchNewEndPoint(end_p, start_p);

    // astar_->SetCenter(odom_p_);
    // bool flag;
    // if ((start_p - end_p).norm() < astar_->resolution_) {
    //     ref_path2_.push_back(start_p);
    //     flag = false;
    // } else {
    //     flag = astar_->SearchPath(start_p, end_p);
    //     if (flag) {
    //         replan_cnt_ = 0;
    //         replan_flag_ = false;
    //         astar_->GetPath(astar_path);
    //         cb_->AstarPublish(astar_path, 0, astar_->resolution_);
    //         assert(astar_path.size() > 0);

    //         ref_path2_.insert(ref_path2_.begin(), astar_path.begin(), astar_path.end());
    //     } else {
    //         replan_cnt_ ++;
    //         ROS_INFO("\033[41;37m No path! Stay at current point! \033[0m");
    //         ref_path2_.push_back(cb_->odom_p_);
    //     }
    // }

    // log_times_[2] = (ros::Time::now() - t0).toSec();
    // return flag;
    return true;
}

void IPCFSMClass::GenerateAPolytope(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes)
{
    planes.resize(0, 4);
    Eigen::Vector3d box_max(1.5, 1.5, 1.0), box_min(-1.5, -1.5, -1.0);
    Eigen::Matrix<double, 6, 4> bd;
    bd.setZero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;
    bd(0, 3) = -p1.x()-box_max.x();
    bd(1, 3) =  p1.x()+box_min.x();
    bd(2, 3) = -p1.y()-box_max.y();
    bd(3, 3) =  p1.y()+box_min.y();
    bd(4, 3) = -p1.z()-box_max.z();
    bd(5, 3) =  p1.z()+box_min.z();

    geometry_utils::Polytope p;
    if (pc_.empty()) { // 障碍物点云为空，直接返回一个方块
        planes.resize(6, 4); // Ax + By + Cz + D = 0
        planes.row(0) <<  1,  0,  0, -p1.x()-box_max.x();
        planes.row(1) <<  0,  1,  0, -p1.y()-box_max.y();
        planes.row(2) <<  0,  0,  1, -p1.z()-box_max.z();
        planes.row(3) << -1,  0,  0,  p1.x()+box_min.x();
        planes.row(4) <<  0, -1,  0,  p1.y()+box_min.y();
        planes.row(5) <<  0,  0, -1,  p1.z()+box_min.z();
        p.SetPlanes(planes);
        p.Visualize(cb_->sfc_pub_, "sfc", 0, Color::SteelBlue(), Color::Gray(), Color::Red(), 0.15, 0.05);
        return ; 
    }
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(pc_[0].data(), 3, pc_.size());

    RET_CODE success = ciri_.comvexDecomposition(bd, pp, pc_rs_, p1, p2);
    if (success == type_utils::SUCCESS) {
        ciri_.getPolytope(p);
        p.Visualize(cb_->sfc_pub_, "sfc", 0, Color::SteelBlue(), Color::Gray(), Color::Red(), 0.15, 0.05);
        planes = p.GetPlanes();
    }
}

}
