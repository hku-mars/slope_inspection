#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <mavros_msgs/RCIn.h>

ros::Publisher rc_pub;
mavros_msgs::RCIn rc_msg;
bool rc_flag = false;

std::vector<double> joy_axes;
std::vector<int> joy_buttoms;

void ChannelRemap(const int joy_id, const int rc_id)
{
    rc_msg.channels[rc_id] = joy_axes[joy_id] * 450 + 1500;
}

void TriggerRemap(const int joy_id, const int rc_id)
{
    rc_msg.channels[rc_id] = 1950 - joy_axes[joy_id] * 900;
}

void FlightRemap(const int joy_id, const int rc_id)
{
    static int last_buttom = 0, mode = 0;
    if (last_buttom == 0 && joy_buttoms[joy_id] == 1) {
        mode++;
        if (mode >= 3) {
            mode = 0;
        }
    }
    if (mode == 0) rc_msg.channels[rc_id] = 1950;
    else if(mode == 1) rc_msg.channels[rc_id] = 1050;
    else if(mode == 2) rc_msg.channels[rc_id] = 1275;
    
    last_buttom = joy_buttoms[joy_id];
}

void JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    rc_flag = true;
    joy_axes.resize(msg->axes.size());
    joy_buttoms.resize(msg->buttons.size());

    for (int i = 0; i < msg->axes.size(); i++) {
        joy_axes[i] = msg->axes[i];
    }
    for (int i = 0; i < msg->buttons.size(); i++) {
        joy_buttoms[i] = msg->buttons[i];
    }

    ChannelRemap(0, 3);
    ChannelRemap(1, 2);
    ChannelRemap(3, 0);
    ChannelRemap(4, 1);
    FlightRemap(4, 5);
    TriggerRemap(5, 10);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BT_X1_node");
    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoyCallback, ros::TransportHints().tcpNoDelay());
    rc_pub = nh.advertise<mavros_msgs::RCIn>("rc", 1);

    rc_msg.channels.resize(12);
    for (int i = 0; i < rc_msg.channels.size(); i++) {
        rc_msg.channels[i] = 1050;
    }

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        if (rc_flag) {
            rc_msg.header.stamp = ros::Time::now();
            rc_pub.publish(rc_msg);
        }
        rate.sleep();
    }

    return 0;
}