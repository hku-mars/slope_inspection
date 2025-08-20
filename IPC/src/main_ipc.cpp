#include <ros/ros.h>

#include "ipc_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ipc_node");
    ros::NodeHandle nh;

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ipc::IPCFSMClass ipc(nh);

    // ros::spin();

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}