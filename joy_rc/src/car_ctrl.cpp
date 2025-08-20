#include <ros/ros.h>
#include <tf/tf.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>

ros::Publisher odom_pub;
double px = 2, py = 0, pz, yaw = 0;
double vx = 0, vw = 0, gain_vx, gain_vw;

void JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
    vx = msg->axes[4];
    vw = msg->axes[0];
}

void CarOdomPub(double dt) {
    double v = vx * gain_vx;
    yaw += vw * gain_vw * dt;
    px += v * std::cos(yaw) * dt;
    py += v * std::sin(yaw) * dt;
    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = px;
    msg.pose.pose.position.y = py;
    msg.pose.pose.position.z = pz;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    msg.twist.twist.linear.x = v * std::cos(yaw);
    msg.twist.twist.linear.y = v * std::sin(yaw);
    msg.twist.twist.linear.z = 0;
    odom_pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "car_ctrl_node");
    ros::NodeHandle nh;

    std::string node_name = ros::this_node::getName();
    nh.param(node_name + "/gain_vx", gain_vx, 1.0);
    nh.param(node_name + "/gain_vw", gain_vw, 1.0);
    nh.param(node_name + "/pz", pz, 0.5);

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoyCallback, ros::TransportHints().tcpNoDelay());
    odom_pub = nh.advertise<nav_msgs::Odometry>("/car/odom", 10);

    ros::Rate rate(30);
    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();
        ros::Time t_now = ros::Time::now();
        double dt = (t_now - last_time).toSec();
        last_time = t_now;
        CarOdomPub(dt);
        rate.sleep();
    }

    return 0;
}