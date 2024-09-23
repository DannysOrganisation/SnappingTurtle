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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>


class Odom : public rclcpp::Node
{
    public:
        Odom();
        ~Odom();

        /**
        * @brief this function gets the current yaw of the robot
         */
        double get_robot_pose() const;

    private:

        /**
        * @brief this function has the robot drive straight at a slow speed
        * 
        * @param msg is the message received via the topic which odom_sub_
         * is subscribed to
         */
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // Robot Pose Publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odom_pub_; 
        rclcpp::TimerBase::SharedPtr update_timer_;
        void update_pose();

        // Path Publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 

        // Member variables
  
        // Odometry subscriber
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // member variable to store the current yaw
        double robot_pose_;
        
        // member variable to store the path for later viewing
        nav_msgs::msg::Path path_;

};