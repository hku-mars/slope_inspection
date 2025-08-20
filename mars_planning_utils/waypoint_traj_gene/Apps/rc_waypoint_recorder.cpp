//
// Created by yunfan on 2021/10/12.
//


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/RCIn.h"
#include "type_utils/common_type_name.h"
#include <string>
#include <fstream>
#include "ros/package.h"

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "debug_utils/backward.hpp"
#include "geometry_utils/geometry_utils.h"
using namespace geometry_utils;
using namespace type_utils;
using std::vector;

namespace backward {
    backward::SignalHandling sh;
}

/* Define global variables */
ros::Subscriber click_sub;
ros::Subscriber rc_sub;
ros::Subscriber odom_sub;
ros::Publisher pid_cmd_pub, mpc_cmd_pub;
ros::Timer cmd_timer;
ros::Publisher mkr_pub;
ros::Time start_time_;
bool trigger{false}, had_traj{false}, had_odom{false};
Vec3f cur_pos;
double cur_yaw;
vector<Vec3f> waypoints;
std::string filename;

void SaveOneWaypoint(Vec3f waypt, double yaw = 0) {
    static int cnt = 0;

    if (cnt == 0) {
        std::ofstream log_file(filename);
        log_file << waypt.transpose()<<" "<<yaw << std::endl;
        printf(" -- [EXIT] \033[32m SAVE ONE POINT SUCCESS! \033[0m\n");
        cnt++;
    } else {
        std::ofstream log_file(filename, std::ios::app);
        log_file << waypt.transpose()<<" "<<yaw << std::endl;
        printf(" -- [EXIT] \033[32m SAVE ONE POINT SUCCESS! \033[0m\n");
    }

}

void MavrosRcCallback(const mavros_msgs::RCInConstPtr &msg) {
    static int last_ch_10 = 1000;
    int ch_10 = msg->channels[9];
    if (last_ch_10 > 1500 && ch_10 < 1500) {
        std::cout<<YELLOW<< " -- [TRAJ] Trigger by RC, record "<< cur_pos.transpose()<<RESET<<std::endl;
        SaveOneWaypoint(cur_pos, cur_yaw);
    }
    last_ch_10 = ch_10;
}

void OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
    cur_pos = Vec3f(msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
    Quatf q(msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
    cur_yaw = geometry_utils::get_yaw_from_quaternion(q);

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "kite");
    ros::NodeHandle nh("~");
    start_time_ = ros::Time::now();
    std::cout<<GREEN " -- [RC-REC] Begin."<<RESET<<std::endl;

    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 10, MavrosRcCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, OdomCallback);
    std::string package_path_ = ros::package::getPath("waypoint_traj_gene");
    time_t t = time(NULL);
    char ch[64] = {0};
    char result[100] = {0};
    strftime(ch, sizeof(ch) - 1, "-%m-%d--%H:%M:%S-", localtime(&t));
    sprintf(result, "%s", ch);
    filename = std::string(result);
    filename = package_path_ + "/data/waypts-" + filename + ".txt";
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();


    ros::waitForShutdown();

    return 0;
}