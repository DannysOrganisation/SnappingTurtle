/*

TODO

Interface should read from /scan

Should store this in a member called distances

it should publish 4 doubles that correspond to the distances in order as

CENTER LEFT RIGHT HARD_RIGHT
0       30  330     90

Check ENUM for more details
*/

#ifndef _LIDAR_HPP
#define _LIDAR_HPP


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

#include "constants.hpp"
#include "std_msgs/msg/string.hpp"


//--CLidar Interface-----------------------------------------------------------
class Lidar : public rclcpp::Node{
    public:
        Lidar();
        ~Lidar();

    private:
        /**
         * @brief this function is the callback for when @lidar_sub_ receives
         * a message from a topic. This function stores the relavent scan angle
         * data into scan_data_ as well as moving the old scan_data into the 
         * prev_scan_data_ member variable
         * 
         * @param msg is the message received via the topic which lidar_sub
         * is subscribed to
         */
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        //lidar subscriber
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

        //store the current and previous LiDar scan information
        std::vector<double> scan_data_;
        std::vector<double> prev_scan_data_;

        // scan angles we are intersted in observing (degrees)
        static constexpr int NUM_ANGLES = 5;

        uint16_t scan_angle[NUM_ANGLES] = {LidarAngles::CENTER_ANGLE,
                                           LidarAngles::LEFT_ANGLE,
                                           LidarAngles::RIGHT_ANGLE,
                                           LidarAngles::HARD_LEFT_ANGLE,
                                           LidarAngles::HARD_RIGHT_ANGLE};

};
#endif