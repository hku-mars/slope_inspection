//
// Created by yunfan on 2021/8/29.
//


#include "rolling_map/rolling_map.hpp"

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "debug_utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

#include "type_utils/common_type_name.hpp"
#include "path_search/astar.h"

rolling_map::RollingMap::Ptr rm_ptr;
path_search::Astar::Ptr astar_saerch;
ros::Publisher path_pub;

void ClickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    print(fg(color::lime_green), " -- [Rviz] Get click at [{} {} {}]\n", msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z);

    Vec3f click_point(msg->pose.position.x,
                      msg->pose.position.y,
                      1.5);
    static Vec3f click_one;
    static int id = 0;

    if (id == 0) {
        click_one = click_point;
        id++;
    } else {
        id = 0;
        int flag = path_search::UNKNOWN_AS_FREE | path_search::ON_INF_MAP;
        vec_Vec3f path;
        astar_saerch->PointToPointPathSearch(click_one, click_point,flag,-1, path);
        print(fg(color::light_sea_green), " -- [RM-Test] Path size: {}\n", path.size());
        VisualUtils::VisualizePath(path_pub, path,Color::SteelBlue(),"astar_path",0.2,0.1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");
    path_pub = nh.advertise<visualization_msgs::MarkerArray>("/astar_path", 1);
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    print(fg(color::light_sea_green), " -- [RM-Test] Begin.\n");
    rolling_map::RollingMapConfig rm_cfg(nh);
    rm_ptr.reset(new rolling_map::RollingMap(nh, rm_cfg));
    astar_saerch.reset(new path_search::Astar);
    astar_saerch->initMap(nh, rm_ptr);
    ros::Subscriber click_sub = nh.subscribe("/goal", 1000, ClickCallback);
    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

