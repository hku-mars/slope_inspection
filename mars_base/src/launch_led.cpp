#include <ros/ros.h>

#include "mars_base/WS2812.h"

#include <mavros_msgs/RCIn.h>
#include <std_msgs/Int8.h>

ros::Publisher led_pub, buzzer_pub;
std_msgs::Int8 buzzer_msg;

void RCCallback(const mavros_msgs::RCInConstPtr& msg) {
    if (msg->channels[6] > 1500) {
        buzzer_msg.data = 1;
    } else {
        buzzer_msg.data = 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "launch_led_node");
    ros::NodeHandle nh;

    led_pub    = nh.advertise<mars_base::WS2812>("/mars/LED", 1);
    buzzer_pub = nh.advertise<std_msgs::Int8>("/mars/buzzer", 1);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, &RCCallback, ros::TransportHints().tcpNoDelay());
    ros::Rate(0.5).sleep();

    buzzer_msg.data = 0;
    
    mars_base::WS2812 msg;
    mars_base::WSRGB rgb;

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        static int count = 0;
        count++;
        if (count == 1) {
            msg.mode = 3;
            msg.channel = 1;
            rgb.r = 255;
            rgb.g = 75;
            rgb.b = 0;
            msg.rgb.clear();
            msg.rgb.push_back(rgb);
        } else if (count == 2) {
            msg.mode = 3;
            msg.channel = 2;
            rgb.r = 255;
            rgb.g = 75;
            rgb.b = 0;
            msg.rgb.clear();
            msg.rgb.push_back(rgb);
        } else if (count == 3) {
            msg.mode = 1;
            msg.channel = 3;
            rgb.r = 0;
            rgb.g = 255;
            rgb.b = 0;
            msg.rgb.clear();
            msg.rgb.push_back(rgb);
        } else if (count == 4) {
            count = 0;
            msg.mode = 1;
            msg.channel = 4;
            rgb.r = 0;
            rgb.g = 255;
            rgb.b = 0;
            msg.rgb.clear();
            msg.rgb.push_back(rgb);
        }
        led_pub.publish(msg);
        buzzer_pub.publish(buzzer_msg);
        rate.sleep();
    }

    return 0;
}