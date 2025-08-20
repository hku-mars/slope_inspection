#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/TwistStamped.h>

ros::Publisher cmd_pub;

void CmdPublish(double vx, double vy, double vw)
{
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.angular.z = vw;
    cmd_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_ctrl_node");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

    double vx = 0, vy = 0, vw = 0;
    double gain1, gain2;
    nh.param("/key_ctrl_node/gain1", gain1, 1.0);
    nh.param("/key_ctrl_node/gain2", gain2, 1.0);
    
    cv::Mat ctrl_img(600, 600, CV_8UC1);
    cv::namedWindow("robot_ctrl");
    cv::moveWindow("robot_ctrl", 200, 0);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        cv::imshow("robot_ctrl", ctrl_img);
        char key = cv::waitKey(1);
        if (key == 'w') {
            vx += 0.01 * gain1;
        }
        if (key == 's') {
            vx -= 0.01 * gain1;
        }
        if (key == 'a') {
            vy += 0.01 * gain1;
        }
        if (key == 'd') {
            vy -= 0.01 * gain1;
        }
        if (key == 'q') {
            vw += 0.01 * gain2;
        }
        if (key == 'e') {
            vw -= 0.01 * gain2;
        }
        
        if (key == 'g') {
            vx = 0;
            vy = 0;
            vw = 0;
        }
        // if (vx > 0.3) vx = 0.3;
        // if (vx <-0.3) vx =-0.3;
        // if (vw > 0.3) vw = 0.3;
        // if (vw <-0.3) vw =-0.3;
        // std::cout << "vx: " << vx << " vw: " << vw << std::endl;

        CmdPublish(vx, vy, vw);

        rate.sleep();
    }
    
    

    return 0;
}
