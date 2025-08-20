#include <ros/ros.h>

#include "mars_led.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mars_led_test_node");
    ros::NodeHandle nh;

    MARSLEDClass mars_led(nh);

    ros::Rate(1).sleep();
    mars_led.WS2812RGB(0, 255, 0, 0);
    ros::Rate(1).sleep();
    mars_led.WS2812Flowing(1, 0, 255, 0);
    ros::Rate(1).sleep();
    mars_led.WS2812RGB(2, 0, 0, 255);
    ros::Rate(1).sleep();
    uint8_t r[5] = {255, 0, 0, 0, 0};
    uint8_t b[5] = {0, 255, 0, 0, 255};
    uint8_t g[5] = {0, 0, 255, 0, 255};
    mars_led.WS2812RGBs(3, 5, r, g, b);
    ros::Rate(1).sleep();
    mars_led.WS2812Breathing(4, 255, 0, 0);
    ros::Rate(1).sleep();

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        // mars_led.WS2812Breathing(0, 0, 255, 0);
        // mars_led.WS2812RGB(0, 0, 0, 255);
        mars_led.WS2812Flowing(0, 0, 0, 255);
        rate.sleep();
    }
    
    return 0;
}