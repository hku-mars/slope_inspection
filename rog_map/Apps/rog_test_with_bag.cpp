
//
// Created by yunfan on 2021/8/29.
//


#include <type_utils/common_type_name.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rog_map/rog_map.h>

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "debug_utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

rosbag::Bag bag;

using namespace std;
using namespace rog_map;

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    cout << GREEN " -- [RM-Test] Begin." << endl;

    /* Callback functions begin */
    string bag_name;
    nh.getParam("bag_name", bag_name);
    cout << BLUE << " -- [RM-Test] Bag name: " << bag_name << RESET << endl;
    bag.open(bag_name, rosbag::bagmode::Read);
    vector<string> topics;
    string odom_topic = "/lidar_slam/odom";
    string lidar_topic = "/cloud_registered";
    topics.push_back(odom_topic);
    topics.push_back(lidar_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rog_map::ROGMap::Ptr rog_map_ptr;
    rog_map::ROGMapConfig cfg;
    rog_map::ROSParamLoader(nh, cfg);
    rog_map_ptr.reset(new rog_map::ROGMap(nh, cfg));
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    int bag_size = view.size();
    int count = 0;
    bool have_odom{false};
    type_utils::Pose pose;
    for (rosbag::MessageInstance const m: view) {
        count++;
        cout << " -- [SCK] Read bag message: " << count << "/ " << bag_size << " " << m.getTopic() << endl;

        if (m.getTopic() == odom_topic) {
            nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
            if (odom_msg != nullptr) {
                have_odom = true;
                pose.first = (Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                                              odom_msg->pose.pose.position.z));
                pose.second = (Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                                                  odom_msg->pose.pose.orientation.y,
                                                  odom_msg->pose.pose.orientation.z));

            } else {
                cout << YELLOW << " Read nullptr." << RESET << endl;
            }
        } else if (m.getTopic() == lidar_topic) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg != nullptr) {
                if (!have_odom) { continue; }
                static pcl::PointCloud<pcl::PointXYZINormal> latest_cloud;
                pcl::fromROSMsg(*cloud_msg, latest_cloud);
                rog_map_ptr->updateMap(latest_cloud, pose);
            } else {
                cout << YELLOW << " Read nullptr." << RESET << endl;
            }
        } else {
            cout << " -- [SCK] Read outlier topic: " << m.getTopic() << endl;
        }
    }
    cout << " -- [SCK] Read bag message finished, waitting for shutdown" << endl;
    while (ros::ok()) {

    }
    return 0;
}

