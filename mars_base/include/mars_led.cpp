#include "mars_led.hpp"

void MARSLEDClass::TimerCallbcak(const ros::TimerEvent & e)
{
    timer_mutex_.lock();
    
    ros::Time t0 = ros::Time::now();
    size_t len = ser_.available();
    if (len >= 500) ROS_WARN("[robot_base] Too Long Message Received! Message length: %ld", len);
    if (len) {
        ser_.read(recv_buffer_, len);
        if (recv_buffer_[0] == 0xAA) {
            mars_base::BuckParam msg;
            memcpy(&msg.vin,  &recv_buffer_[1], 4);
            memcpy(&msg.vout, &recv_buffer_[5], 4);
            memcpy(&msg.temp, &recv_buffer_[9], 4);
            buck_pub_.publish(msg);
            last_recv_time_ = t0;
        }
    }
    if ((t0 - last_recv_time_).toSec() > 2.0) ROS_ERROR("[robot_base]: Check connection!!!");

    timer_mutex_.unlock();
}

void MARSLEDClass::SetAllRGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (!init_flag_) return;
    WS2812RGB(0, r, g, b);
}

void MARSLEDClass::WS2812RGB(uint8_t channel, uint8_t r, uint8_t g, uint8_t b)
{
    if (!init_flag_) return;
    uint8_t buffer[100];
    uint8_t send_data_len = 5;
    buffer[0] = 0xAA;
    buffer[1] = ((buzzer_mode_ << 6) & 0xC0) | (channel & 0x3F);
    buffer[2] = r;
    buffer[3] = g;
    buffer[4] = b;
    if ((ros::Time::now() - last_time_).toSec() >= 0.001) {
        last_time_ = ros::Time::now();
        ser_.write(buffer, send_data_len);
    }
}

void MARSLEDClass::WS2812RGBs(uint8_t channel, uint8_t lights, uint8_t* r, uint8_t* g, uint8_t* b)
{
    if (!init_flag_) return;
    uint8_t buffer[100];
    uint8_t send_data_len = 2 + 3*lights;
    buffer[0] = 0xAA;
    buffer[1] = ((buzzer_mode_ << 6) & 0xC0) | ((channel + 10) & 0x3F);
    for (int i = 0; i < lights; i++) {
        buffer[2 + i*3] = r[i];
        buffer[3 + i*3] = g[i];
        buffer[4 + i*3] = b[i];
    }
    if ((ros::Time::now() - last_time_).toSec() >= 0.001) {
        last_time_ = ros::Time::now();
        ser_.write(buffer, send_data_len);
    }
}

void MARSLEDClass::WS2812Breathing(uint8_t channel, uint8_t r, uint8_t g, uint8_t b)
{
    if (!init_flag_) return;
    uint8_t buffer[100];
    uint8_t send_data_len = 5;
    buffer[0] = 0xAA;
    buffer[1] = ((buzzer_mode_ << 6) & 0xC0) | ((channel + 20) & 0x3F);
    buffer[2] = r;
    buffer[3] = g;
    buffer[4] = b;
    if ((ros::Time::now() - last_time_).toSec() >= 0.001) {
        last_time_ = ros::Time::now();
        ser_.write(buffer, send_data_len);
    }
}

void MARSLEDClass::WS2812Flowing(uint8_t channel, uint8_t r, uint8_t g, uint8_t b)
{
    if (!init_flag_) return;
    uint8_t buffer[100];
    uint8_t send_data_len = 5;
    buffer[0] = 0xAA;
    buffer[1] = ((buzzer_mode_ << 6) & 0xC0) | ((channel + 30) & 0x3F);
    buffer[2] = r;
    buffer[3] = g;
    buffer[4] = b;
    if ((ros::Time::now() - last_time_).toSec() >= 0.001) {
        last_time_ = ros::Time::now();
        ser_.write(buffer, send_data_len);
    }
}
