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
#include <geometry_msgs/msg/twist.hpp>


#include "enum.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 0.2

class Motordrive : public rclcpp::Node{
    public:
        Motordrive();
        ~Motordrive();
    private:
        
        /**
        * @brief this function is called every time a message is received from
        * /state topic and it calls the appropriate function for each state
        *
        * @param msg is an 32 bit integer which represents the current state 
        */
        void state_callback(const std_msgs::msg::Int32 msg);

        /**
        *@brief this function publishes to /cmd_vel topic with the desired 
        * velocity for the turtlebot
        *
        * @param linear is the linear velocity in m/s
        * @param angular is the angular velocity in rad/s 
         */
        void update_cmd_vel(double linear, double angular);

        /**
        * @brief this function has the robot turn left on the spot
         */
        void turn_left();

        /**
        * @brief this function has the robot turn right on the spot
         */
        void turn_right();

        /**
        * @brief this function has the robot turn left in an arc
         */
        void turn_hard_left();

        /**
        * @brief this function has the robot turn right in an arc
         */
        void turn_hard_right();

        /**
        * @brief this function has the robot drive straight at a desired speed
         */
        void drive_forward();

        /**
        * @brief this function has the robot drive straight at a slow speed
         */
        void slow_forward();

        //memory of the current state
        int current_state_;

        //publish to cmd_vel and subscribe to state
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motor_driver_sub_;
};

#endif