#include <rog_map/rog_map.h>

int main(int argc, char **argv) {
    cerr << "This is a test file." << endl;
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    rog_map::ROGMapConfig rm_cfg;
    rog_map::ROSParamLoader(nh, rm_cfg);
    rog_map::ROGMap::Ptr rog_ptr;
    rog_ptr.reset(new rog_map::ROGMap(nh, rm_cfg));
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

