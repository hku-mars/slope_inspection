//
// Created by yunfan on 2021/8/29.
//

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/ColorRGBA.h"
#include "type_utils/common_type_name.h"
#include "visualization_utils/visualization_utils.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"
#include "nav_msgs/Odometry.h"

using namespace type_utils;
using namespace visualization_utils;
using std::string;
using std::vector;
using std::cout;
using std::endl;


static void VisualizeOdom(const ros::Publisher &pub_,
                          const vec_Vec3f &posis_,
                          const vec_E<Quatf> &quats_,
                          const double &tor_dis,
                          const double &scale,
                          string mesh_path) {
    for (long unsigned int i = 0; i < posis_.size(); i++) {
        VisualUtils::VisualizeDrone(pub_, posis_[i], quats_[i], mesh_path, "drone", scale, i);
        ros::Duration(0.02).sleep();
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "draw_odom");
    ros::NodeHandle nh("~");
    double scale, pos_tor;
    string bag_name, lidar_topic, mesh_resource;

    nh.getParam("bag_path", bag_name);
    nh.getParam("scale", scale);
    nh.getParam("pos_tor", pos_tor);
    nh.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/f250.dae"));
    cout<<GREEN<< " -- [SCK] Bag path: "<< bag_name<<RESET<<endl;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    vec_Vec3f positions;
    vec_E<Quatf> quats;
    rosbag::Bag bag;
    bag.open(bag_name, rosbag::bagmode::Read);
    vector<string> topics;
    string odom_topic = "/lidar_slam/odom";
    topics.push_back(odom_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int bag_size = view.size();
    int count = 0;
    cout<< " -- [SCK] Bag size: "<<bag_size<< " total "<<bag.getSize()<<RESET<<endl;
    for (rosbag::MessageInstance const m: view) {
        count++;
        cout<<YELLOW<< " -- [SCK] Read bag message: "<<count<<" / "<<bag_size<<" "<< m.getTopic()<<RESET<<endl;
        if (m.getTopic() == odom_topic) {
            nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
            if (odom_msg != nullptr) {
                Vec3f posi = Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                                             odom_msg->pose.pose.position.z);
                Quatf quatf = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                                                 odom_msg->pose.pose.orientation.y,
                                                 odom_msg->pose.pose.orientation.z);
                static Vec3f last_posi(999, 999, 999);
                if ((last_posi - posi).norm() > pos_tor) {
                    last_posi = posi;
                    positions.push_back(posi);
                    quats.push_back(quatf);
                }
            } else {
                cout<<YELLOW<< "nullptr"<<RESET<<endl;
            }
        } else {
            cout<<" -- [SCK] Read outlier topic: "<< m.getTopic()<<endl;
        }
    }
    ros::Duration(1.0).sleep();

    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("odom_mesh", 1);

    VisualizeOdom(pub, positions, quats, pos_tor, scale, mesh_resource);

    ros::waitForShutdown();

    return 0;
}
