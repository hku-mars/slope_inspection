#include <ros/ros.h>

#include <mavros_msgs/RCIn.h>
#include <serial/serial.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sbus_rc_node");
    ros::NodeHandle nh;

    ros::Publisher rc_pub = nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in", 10);

    serial::Serial ser;
    int ser_rate = 115200;
    std::string port = "/dev/ttyUSB0";
    if (port.empty()) {
        ROS_WARN("Serial port error!");
        return 1;
    } 
    try {
        ser.setPort(port);
        ser.setBaudrate(ser_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch(serial::IOException& e) {
        ROS_ERROR("Unable to open port: %s", port.data());
        return 1;
    }
    if (ser.isOpen()) ROS_INFO("\033[32mSerial port %s initialized!\033[32m", port.data());
    else return 1;

    int min_rc = 290, max_rc = 1730;
    ros::Rate rate(1000);
    uint8_t recv_buffer[500], cur_buffer[100];
    while (ros::ok()) {
        ros::spinOnce();
        int ret = ser.read(cur_buffer, 1);
        int buf_ptr_offset = 0;
        if (ret > 0 && cur_buffer[0] == 0x0f) {
            recv_buffer[0] = 0x0f;
            while (buf_ptr_offset < 34) {
                ret = ser.read(cur_buffer, 1);
                if (ret > 0) {
                    recv_buffer[buf_ptr_offset + 1] = cur_buffer[0];
                    buf_ptr_offset += 1;
                }
            }
            uint8_t var = recv_buffer[1];
            for (int i = 2; i <= 33; i++) {
                var ^= recv_buffer[i];
            }
            if (var == recv_buffer[34]) {
                mavros_msgs::RCIn msg;
                msg.header.stamp = ros::Time::now();
                uint16_t channel[16];
                for (int i = 0; i < 16; i++) {
                    channel[i] = (recv_buffer[2 * i + 1] << 8) | recv_buffer[2 * i + 2];
                    channel[i] = 1000 + 1000 * (channel[i] - min_rc) / (max_rc - min_rc);
                }
                msg.channels = std::vector<uint16_t>(channel, channel + 16);
                rc_pub.publish(msg);
            }
        }
        rate.sleep();
    }
    ser.close();

    return 0;
}