//
// Created by yunfan on 2021/8/29.
//


#include "rog_map/rog_map.h"



/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "debug_utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}


rog_map::ROGMap::Ptr rm_ptr;


int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    cout<<GREEN" -- [RM-Test] Begin."<<endl;
    rog_map::ROGMapConfig cfg;
    rog_map::ROSParamLoader ld(nh, cfg);
    rm_ptr.reset(new rog_map::ROGMap(nh, cfg));

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

