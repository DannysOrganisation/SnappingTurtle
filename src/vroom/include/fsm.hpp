/*
TODO
State machine
States should be set out as follows in the enum file
Work out this based off the turtlebot3_drive.cpp

This should have the lidar, cam and odom as member nodes in side of this node

/<whatever james does with the camera lol>

This should publish to:
/state

*/

#include "constants.hpp"
#include "lidar.hpp"
#include "odom.hpp" 
// #include "CameraReader.hpp"
#include "std_msgs/msg/int32.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class FSM : public rclcpp::Node
{

    public:
        FSM();
        ~FSM();

    
    private:
        // ROS topic publishers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;

        // callback function 
        void update_state();

        // timer to control how often state gets published
        rclcpp::TimerBase::SharedPtr update_timer_;

        // the current state of the system
        int current_state_;

        // Nodes that control reading from the data itself

        // Instances of the child nodes
        std::shared_ptr<Odom> odom_node_;
        std::shared_ptr<Lidar> lidar_node_;
        // std::shared_ptr<CameraReader> camera_reader_node_;


        // wall finding member variables
        double min_distance;
        double min_distance_pose;

        // track poses that need to be remembe
        double prev_robot_pose_;

        std::vector<double> scan_data_;
        std::vector<double> prev_scan_data_;
};