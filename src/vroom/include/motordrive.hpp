/*

TODO 

This should control the drive based on the current state.

It should subscribe to the /state topic
it should publish to the /cmd_vel topic
*/

#ifndef _MOTORDRIVE_HPP
#define _MOTORDRIVE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

#include "enum.h"
#include "std_msgs/msg/string.hpp"

class Motordrive : public rclcpp::Node{
    public:
        CMotordrive();
        ~CMotordrive();

    private:
        void fsm_callback(const std_msg::msg::Int32 msg);

        void turn_left();
        void turn_right();
        void turn_hard_left();
        void turn_hard_right();
        void drive_forward();
        void slow_forward();

        int current_state_;

        rclcpp::Subscription<std_msg::msg::Int32>::SharedPtr motore_driver_sub_;


}

#endif