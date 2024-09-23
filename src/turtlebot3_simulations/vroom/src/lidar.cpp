/*
lidar.cpp IMPLEMENTATION

This is the implementation for the Lidar node which reads from the
turtlebot lidar and publishes specific relevant scan angles required
for driving.

Written: Adam Riesel
Edited: Daniel Monteiro
*/

#include "lidar.hpp"

using namespace std::chrono_literals;

//--Lidar Implementation--------------------------------------------
Lidar::Lidar() : Node ("tb3_LiDar")
{ 
    //create LiDar subscriber
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", /* subscribe to topic /scan */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &Lidar::scan_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

    //resize scan data member variables
    scan_data_.resize(LidarAngles::NUM_ANGLES);
    prev_scan_data_.resize(LidarAngles::NUM_ANGLES);   


    // initialise publisher
    scan_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("lidar", STANDARD_BUFFER_SIZE);
    // create the timer that will cotrol how often the state gets published
    update_timer_ = this->create_wall_timer(20ms, std::bind(&Lidar::update_scan_data, this));

    RCLCPP_INFO(this->get_logger(), "Lidar_node has been successfully initialised");


}

//--
Lidar::~Lidar()
{
    RCLCPP_INFO(this->get_logger(), "Lidar_node has been terminated");
}

//--
void Lidar::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    //save previous scan data into member variable
    prev_scan_data_ = scan_data_;

    for (int num = 0; num < LidarAngles::NUM_ANGLES; num++) {
        if (std::isinf(msg->ranges.at(scan_angle[num]))) {
        scan_data_[num] = msg->range_max;
        } else {
        scan_data_[num] = msg->ranges.at(scan_angle[num]);
        }
    }
}


void Lidar::update_scan_data()
{   
    // create the message and insert the scan data into it
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.insert(message.data.end(), scan_data_.begin(), scan_data_.end());

    // publish the message
    scan_data_pub_->publish(message);
}




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lidar>());
    rclcpp::shutdown();
}