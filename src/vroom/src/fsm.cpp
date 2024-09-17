/*

FSM CLASS IMPLEMENTATION

*/

#include "vroom/fsm.hpp"

using namespace std::chrono_literals;

FSM::FSM(): Node("fsm_node"), current_state_(0)
{
    // Create instances of Lidar, Odom, and Cam nodes
    lidar_node_ = std::make_shared<Lidar>();
    odom_node_ = std::make_shared<Odom>();
    cam_node_ = std::make_shared<Cam>();

    // create the state publisher
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/state", 10);

    // create the timer that will cotrol how often the state gets published
    update_timer_ = this->create_wall_timer(
            10ms, std::bind(&FSM::update_state, this));
}

FSM::~FSM()
{
    RCLCPP_INFO(this->get_logger(), "FSM node has been terminated");
}


void FSM::update_state()
{

    // Create an integer message
    auto msg = std_msgs::msg::Int32();
    msg.data = current_state_;

    // Publish the message
    state_pub_->publish(msg);

    // Log the current state
    RCLCPP_INFO(this->get_logger(), "Published state: %d", msg.data);


    /*
    TO DO

    WORK OUT THE NEXT STATE LOGIC AND PROCESS IT HERE
    */

}

