/*
CameraReader.hpp

This header file declares the class for the node that will 
subscribe to the raw image and will process the data. It will
then find the density of RGB values in the image, allowing 
us to find when to 'finish the maze'

James Hocking, 2024
*/

#ifndef _CAMERA_READER_HPP   
#define _CAMERA_READER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include "constants.hpp"

class CameraReader : public rclcpp::Node {         
    public:
        
        CameraReader();  
        ~CameraReader();  

        /**
         * @brief This function gets the current percentage of pixels
         * that are red in the camera vision
         */
        float get_last_r_density() const;

        /**
         * @brief This function gets the current percentage of pixels
         * that are green in the camera vision
         */
        float get_last_g_density() const;

        /**
         * @brief This function gets the current percentage of pixels
         * that are blue in the camera vision
         */
        float get_last_b_density() const;
        
    private:

        // Variables to store the last density of colors
        float last_r_density_ = 0.0f;
        float last_g_density_ = 0.0f;
        float last_b_density_ = 0.0f;
        
        // Subscriber to the camera available on the turtlebot
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

        /**
         * @brief this function is the callback for when @camera_sub_ receives
         * a frame from the topic. Callback loops through each pixel and 
         * sums the rgb values then stores the percentage of each value
         * relative to the total sum.
         * 
         * @param msg is the message received via the topic which camera_sub_
         * is subscribed to
         */
        void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);        

        // Publisher of the green density for finishing
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_pub_;

        // Publisher of only the center green density for tracking
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr\
        center_green_density_pub_;
};

#endif  