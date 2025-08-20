#include <ros/ros.h>

#include "mars_base/WS2812.h"
#include <std_msgs/Int8.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ws2812_ctrl_node");
    ros::NodeHandle nh;

    ros::Publisher pub1 = nh.advertise<mars_base::WS2812>("/mars/LED", 1);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int8>("/mars/buzzer", 1);
    ros::Rate(0.5).sleep();

    std_msgs::Int8 buzzer_msg;
    buzzer_msg.data = 1;
    pub2.publish(buzzer_msg);
    
    mars_base::WS2812 msg;
    mars_base::WSRGB rgb;
    msg.mode = 1;
    msg.channel = 1;
    msg.lights = 5;
    rgb.r = 255;
    msg.rgb.push_back(rgb);
    pub1.publish(msg);
    pub2.publish(buzzer_msg);
    ros::Rate(1).sleep();

    msg.mode = 2;
    msg.channel = 2;
    msg.lights = 5;
    msg.rgb.clear();
    msg.rgb.push_back(rgb);
    rgb.g = 0;
    rgb.r = 255;
    msg.rgb.push_back(rgb);
    rgb.r = 0;
    rgb.b = 255;
    msg.rgb.push_back(rgb);
    pub1.publish(msg);
    pub2.publish(buzzer_msg);
    ros::Rate(1).sleep();

    msg.mode = 3;
    msg.channel = 3;
    msg.lights = 5;
    msg.rgb.clear();
    rgb.r = 0;
    rgb.g = 0;
    rgb.b = 255;
    msg.rgb.push_back(rgb);
    pub1.publish(msg);
    buzzer_msg.data = 0;
    pub2.publish(buzzer_msg);
    ros::Rate(1).sleep();

    msg.mode = 4;
    msg.channel = 4;
    msg.lights = 5;
    msg.rgb.clear();
    rgb.r = 0;
    rgb.g = 255;
    rgb.b = 0;
    msg.rgb.push_back(rgb);
    pub1.publish(msg);
    buzzer_msg.data = 0;
    pub2.publish(buzzer_msg);
    ros::Rate(1).sleep();

    return 0;
}