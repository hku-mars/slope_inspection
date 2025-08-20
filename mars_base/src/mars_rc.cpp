#include <ros/ros.h>

#include <serial/serial.h>

#include <mavros_msgs/RCIn.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mars_rc_node");
    ros::NodeHandle nh;

    ros::Publisher rc_pub = nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in", 10);

    serial::Serial ser;
    int ser_rate = 921600;
    std::string port = "/dev/Critical_HIT_Robot";
    if (port.empty()) {
        ROS_WARN("[MARS_base] Serial port error!");
        return 1;
    } 
    try {
        ser.setPort(port);
        ser.setBaudrate(ser_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch(serial::IOException& e) {
        ROS_ERROR("[MARS_base] Unable to open port: %s", port.data());
        return 1;
    }
    if (ser.isOpen()) ROS_INFO("\033[32m[MARS_base] Serial port %s initialized!\033[32m", port.data());
    else return 1;
    
    ros::Rate rate(1000);
    uint8_t recv_buffer[500];
    while (ros::ok()) {
        ros::spinOnce();
        size_t len = ser.available();
        if (len) {
            ser.read(recv_buffer, len);
            if (recv_buffer[0] == 0xAA) {
                mavros_msgs::RCIn msg;
                msg.header.stamp = ros::Time::now();
                for (int i = 0; i < 12; i++) {
                    int16_t data = (int16_t)((recv_buffer[(i+1)*2] << 8) | recv_buffer[(i+1)*2-1]);
                    msg.channels.push_back((uint16_t)(1500 + (data * 450.0 / 700.0)));
                }
                for (int i = 12; i < 16; i++) {
                    msg.channels.push_back(1500);
                }
                rc_pub.publish(msg);
            }
        }
        rate.sleep();
    }
    ser.close();

    return 0;
}