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
#include "rolling_map/astar2.hpp"

rolling_map::RollingMap::Ptr rm_ptr;
astar::Astar::Ptr astar_saerch;


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
        astar_saerch->AstarSearch(astar::Astar::NORMAL, click_one, click_point);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    print(fg(color::light_sea_green), " -- [RM-Test] Begin.\n");
    rolling_map::RollingMapConfig rm_cfg(nh);
    rm_ptr.reset(new rolling_map::RollingMap(nh, rm_cfg));
    astar_saerch.reset(new astar::Astar);
    astar_saerch->initMap(nh, rm_ptr);
    ros::Subscriber click_sub = nh.subscribe("/goal", 1000, ClickCallback);
    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

