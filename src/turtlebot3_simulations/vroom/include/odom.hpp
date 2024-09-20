/*
ODOMETRY NODE INTERFACE

Should contain an interface for the odometry.

Odometry should subscribe to odom and then calculate the robots pose

This should then store this in a member called robot_pose_

*/

#include "constants.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>


class Odom : public rclcpp::Node
{
    public:
        Odom();
        ~Odom();

        // Getters
        double get_robot_pose() const;

    private:

        // Callback function for subscriber
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // Member variables
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        double robot_pose_;
        double prev_robot_pose_;

};