#ifndef MARS_LED_H
#define MARS_LED_H

#include "ros/ros.h"

#include <mutex>
#include <serial/serial.h>

#include <std_msgs/Int8.h>

#include "mars_base/BuckParam.h"
#include "mars_base/WS2812.h"

class MARSLEDClass {
public:
    typedef std::shared_ptr<MARSLEDClass> Ptr;
    MARSLEDClass(ros::NodeHandle& nh, bool callback_flag = false) {
        int serial_rate = 921600;
        std::string serial_port = "/dev/MARS_LED";

        bool flag = SerialInit(&ser_, serial_port, serial_rate);
        if (flag == false) return ;
        init_flag_ = true;
        buzzer_mode_ = 0;
        callback_flag_ = callback_flag;

        led_sub_    = nh.subscribe<mars_base::WS2812>("/mars/LED", 10, &MARSLEDClass::LEDCallback, this, ros::TransportHints().tcpNoDelay());
        buzzer_sub_ = nh.subscribe<std_msgs::Int8>("/mars/buzzer", 10, &MARSLEDClass::BuzzerModeCallback, this, ros::TransportHints().tcpNoDelay());
        buck_pub_   = nh.advertise<mars_base::BuckParam>("/mars/buck_param", 10);

        WS2812RGB(0, 0, 0, 0);
        ros::Rate(10).sleep();
        SetBeepMode(1);
        for (int i = 1; i <= 4; i++) {
            for (int j = 1; j <= 4; j++) {
                WS2812RGB(j, 0, 0, 0);
                ros::Rate(1000).sleep();
            }
            WS2812RGB(i, 255, 0, 0);
            ros::Rate(1).sleep();
        }

        SetBeepMode(0);
        WS2812RGB(0, 0, 0, 0);
        last_time_ = ros::Time::now();

        timer_ = nh.createTimer(ros::Duration(0.001), &MARSLEDClass::TimerCallbcak, this);
    }
    ~MARSLEDClass() {
        SetBeepMode(0);
        WS2812RGB(0, 0, 0, 0);
        ser_.close();
    }

    void WS2812RGB(uint8_t channel, uint8_t r, uint8_t g, uint8_t b);
    void SetAllRGB(uint8_t r, uint8_t g, uint8_t b);
    void WS2812RGBs(uint8_t channel, uint8_t lights, uint8_t* r, uint8_t* g, uint8_t* b);
    void WS2812Breathing(uint8_t channel, uint8_t r, uint8_t g, uint8_t b);
    void WS2812Flowing(uint8_t channel, uint8_t r, uint8_t g, uint8_t b);
    void SetBeepMode(uint8_t mode) {
        if (mode > 3) return ;
        buzzer_mode_ = mode;
    }

    bool init_flag_{false}, send_flag_{false};

private:
    void TimerCallbcak(const ros::TimerEvent & e);

    void BuzzerModeCallback(const std_msgs::Int8ConstPtr& msg) {
        buzzer_mutex_.lock();
        SetBeepMode(msg->data);
        if (msg->data >= 0 && msg->data <= 3) {
            buzzer_mode_ = msg->data;
        }
        buzzer_mutex_.unlock();
    }
    void LEDCallback(const mars_base::WS2812ConstPtr& msg) {
        led_mutex_.lock();
        if (msg->rgb.size() == 0 && msg->mode > 0) return; 
        uint8_t r[msg->rgb.size()], g[msg->rgb.size()], b[msg->rgb.size()];
        switch (msg->mode)
        {
        case 0:
            WS2812RGB(0, 0, 0, 0);
            break;
        case mars_base::WS2812::RGB:
            WS2812RGB(msg->channel, msg->rgb[0].r, msg->rgb[0].g, msg->rgb[0].b);
            break;
        case mars_base::WS2812::RGBs:
            for (int i = 0; i < msg->rgb.size(); i++) {
                r[i] = msg->rgb[i].r;
                g[i] = msg->rgb[i].g;
                b[i] = msg->rgb[i].b;
            }
            WS2812RGBs(msg->channel, msg->lights, r, g, b);
            break;
        case mars_base::WS2812::Breath:
            WS2812Breathing(msg->channel, msg->rgb[0].r, msg->rgb[0].g, msg->rgb[0].b);
            break;
        case mars_base::WS2812::Flow:
            WS2812Flowing(msg->channel, msg->rgb[0].r, msg->rgb[0].g, msg->rgb[0].b);
            break;
        default:
            break;
        }
        led_mutex_.unlock();
    }
    bool SerialInit(serial::Serial *ser, std::string port, int rate) {
        if (port.empty()) {
            ROS_WARN("[MARS_base] Serial port error!");
            return false;
        } 
        try {
            ser->setPort(port);
            ser->setBaudrate(rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser->setTimeout(to);
            ser->open();
        } catch(serial::IOException& e) {
            ROS_ERROR("[MARS_base] Unable to open port: %s", port.data());
            return false;
        }
        if (ser->isOpen()) ROS_INFO("\033[32m[MARS_base] Serial port %s initialized!\033[32m", port.data());
        else return false;
        return true;
    }

    bool callback_flag_;
    serial::Serial ser_;

    ros::Time last_time_, last_recv_time_;
    ros::Subscriber led_sub_, buzzer_sub_;
    ros::Publisher buck_pub_;
    ros::Timer timer_;
    std::mutex timer_mutex_, buzzer_mutex_, led_mutex_;

    uint8_t buzzer_mode_;
    uint8_t r_{0}, g_{0}, b_{0};
    uint8_t recv_buffer_[500];

};

#endif
