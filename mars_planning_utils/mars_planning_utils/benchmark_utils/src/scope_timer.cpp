#include "benchmark_utils/scope_timer.h"

using namespace benchmark_utils;

TimeConsuming::TimeConsuming(std::string msg, int repeat_time) {
    repeat_time_ = repeat_time;
    msg_ = msg;
    tc_start = std::chrono::high_resolution_clock::now();
    has_shown = false;
    print_ = true;
}

TimeConsuming::TimeConsuming(std::string msg, bool print_log) {
    msg_ = msg;
    repeat_time_ = 1;
    print_ = print_log;
    tc_start = std::chrono::high_resolution_clock::now();
    has_shown = false;
}

TimeConsuming::~TimeConsuming() {
    if (!has_shown && enable_ && print_) {
        tc_end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast < std::chrono::duration < double >> (tc_end - tc_start).count();
        double t_us = (double) dt * 1e6 / repeat_time_;
        if (t_us < 1) {
            t_us *= 1000;
            printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
        } else if (t_us > 1e6) {
            t_us /= 1e6;
            printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
        } else if (t_us > 1e3) {
            t_us /= 1e3;
            printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
        } else
            printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
    }
}

void TimeConsuming::set_enbale(bool enable) {
    enable_ = enable;
}

void TimeConsuming::start() {
    tc_start = std::chrono::high_resolution_clock::now();
}

double TimeConsuming::stop() {
    if (!enable_) { return 0; }
    has_shown = true;
    tc_end = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast < std::chrono::duration < double >> (tc_end - tc_start).count();
    if (!print_) {
        return dt;
    }
    double t_us = (double) dt * 1e6 / repeat_time_;
    if (t_us < 1) {
        t_us *= 1000;
        printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
    } else if (t_us > 1e6) {
        t_us /= 1e6;
        printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
    } else if (t_us > 1e3) {
        t_us /= 1e3;
        printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
    } else
        printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
    return dt;
}