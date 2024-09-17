#include "lidar.hpp"

//--CLidar Implementation--------------------------------------------
CLidar::CLidar()
    : Node ("tb3_LiDar")
{

    //create LiDar subscriber
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", /* subscribe to topic /scan */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &CLidar::scan_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

    //resize scan data member variables
    scan_data_.resize(num_angles);
    prev_scan_data_.resize(num_angles);   
}

//--
CLidar::~CLidar(){}

//--
void CLidar::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    //save previous scan data into member variable
    prev_scan_data_ = scan_data_;

    //TODO Delete print statement
    RCLCPP_INFO(this->get_logger(), "I received from a topic! Yay!");

    for (int num = 0; num < 4; num++) {
        if (std::isinf(msg->ranges.at(scan_angle[num]))) {
        scan_data_[num] = msg->range_max;
        } else {
        scan_data_[num] = msg->ranges.at(scan_angle[num]);
        }

        //TODO Delete print statement
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", scan_data_[num]);
    }
}

