//
// Created by yunfan on 2021/8/29.
//

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "debug_utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

#include "type_utils/common_type_name.h"
#include "path_search/astar.h"
#include <rog_map/rog_map.h>
#include "visualization_utils/visualization_utils.h"

rog_map::ROGMap::Ptr rm_ptr;
path_search::Astar::Ptr astar_saerch;
using namespace path_search;
ros::Publisher mkr_pub;
using namespace type_utils;
using namespace visualization_utils;
using namespace rog_map;
using std::cout;
using std::endl;

void ClickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    cout << GREEN << " -- [Rviz] Get click at " << msg->pose.position.x << " " << msg->pose.position.y << " "
         << msg->pose.position.z << RESET << endl;

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
        vec_E<Vec3f> out_path;
        int flag = ON_INF_MAP | UNKNOWN_AS_FREE;
        astar_saerch->PointToPointPathSearch(click_one, click_point,flag, 999, out_path);
        VisualUtils::VecVec3fToMkrArr(mkr_pub, out_path, Color::SteelBlue(), "astar_path", 0.2, 0.1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    cout << GREEN << " -- [RM-Test] Begin." << RESET << endl;
    ROGMapConfig cfg;
    ROSParamLoader(nh, cfg);
    rm_ptr.reset(new rog_map::ROGMap(nh, cfg));
    astar_saerch.reset(new path_search::Astar);
    astar_saerch->initMap(nh, rm_ptr);
    ros::Subscriber click_sub = nh.subscribe("/goal", 1000, ClickCallback);
    mkr_pub = nh.advertise<visualization_msgs::MarkerArray>("/path", 1);
    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

