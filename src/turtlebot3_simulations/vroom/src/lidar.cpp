#include "lidar.hpp"

//--Lidar Implementation--------------------------------------------
Lidar::Lidar() : Node ("tb3_LiDar")
{
    // display successful creation message
    RCLCPP_INFO(this->get_logger(), "Lidar_node has been successfully initialised");
    
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
}

//--
Lidar::~Lidar(){}

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


std::vector<double> Lidar::get_scan_data()
{   
    // create a deep copy as to not change the original data (getter)
    std::vector<double> deep_copy_scan_data = scan_data_;
    return deep_copy_scan_data;
}