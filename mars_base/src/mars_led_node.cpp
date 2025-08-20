#include <ros/ros.h>

#include "mars_led.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mars_led_node");
    ros::NodeHandle nh;

    MARSLEDClass mars_led(nh, true);

    ros::spin();

    return 0;
}