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

#include "enum.h"
#include "std_msgs/msg/string.hpp"


//--CLidar Interface-----------------------------------------------------------
class CLidar : public rclcpp::Node{
    public:
        CLidar();
        ~CLidar();

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

        
        //TO-DO MOVE THIS TO CONSTANTS FILE
        static constexpr int CENTER_ANGLE = 0;
        static constexpr int LEFT_ANGLE = 30;
        static constexpr int  RIGHT_ANGLE = 330;
        static constexpr int  HARD_LEFT_ANGLE = 90;
        static constexpr int  HARD_RIGHT_ANGLE = 270;

        uint16_t scan_angle[NUM_ANGLES] = {CENTER_ANGLE,
                                           LEFT_ANGLE,
                                           RIGHT_ANGLE,
                                           HARD_LEFT_ANGLE,
                                           HARD_RIGHT_ANGLE};

};
#endif