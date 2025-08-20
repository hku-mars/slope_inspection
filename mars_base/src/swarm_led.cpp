#include <ros/ros.h>

#include "mars_led.hpp"
#include <std_msgs/Int8.h>

int8_t command = -2;

void LEDCallback(const std_msgs::Int8ConstPtr& msg)
{
    command = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_led_node");
    ros::NodeHandle nh;

    MARSLEDClass mars_led(nh);
    ros::Subscriber sub = nh.subscribe<std_msgs::Int8>("/connected_teammate_num", 10, LEDCallback, ros::TransportHints().tcpNoDelay());

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        switch (command)
        {
        case -2:
            mars_led.WS2812RGB(0, 0, 0, 0);
            break;
        case -1:
            mars_led.WS2812RGB(0, 255, 0, 0);
            break;
        case 0:
            mars_led.WS2812RGB(0, 255, 75, 0);
            break;
        default:
            mars_led.WS2812RGB(0, 0, 255, 0);
            break;
        }
        rate.sleep();
    }

    return 0;
}