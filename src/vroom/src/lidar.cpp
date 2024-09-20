#include "lidar.hpp"

//--CLidar Implementation--------------------------------------------
Lidar::Lidar()
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
    scan_data_.resize(NUM_ANGLES);
    prev_scan_data_.resize(NUM_ANGLES);   
}

//--
Lidar::~Lidar(){}

//--
void Lidar::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    //save previous scan data into member variable
    prev_scan_data_ = scan_data_;

    for (int num = 0; num < NUM_ANGLES; num++) {
        if (std::isinf(msg->ranges.at(scan_angle[num]))) {
        scan_data_[num] = msg->range_max;
        } else {
        scan_data_[num] = msg->ranges.at(scan_angle[num]);
        }
    }
}

