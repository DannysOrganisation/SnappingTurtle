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
#include "std_msgs/msg/float32.hpp"
#include <visualization_msgs/msg/marker.hpp>


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

        // Odometry Publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odom_pub_; 
        rclcpp::TimerBase::SharedPtr update_timer_;
        void update_pose();

        void update_marker();

        // Member variables
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        double robot_pose_;
        double prev_robot_pose_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

        //marker
        visualization_msgs::msg::Marker marker;


};