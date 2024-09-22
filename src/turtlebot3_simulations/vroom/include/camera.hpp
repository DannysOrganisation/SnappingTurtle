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
        // Constructor
        CameraReader();  

        // Destructor
        ~CameraReader();  

        // Getters for the color values
        float get_last_r_density() const;
        float get_last_g_density() const;
        float get_last_b_density() const;
        
    private:
        // Variables to store the last density of colors
        float last_r_density_ = 0.0f;
        float last_g_density_ = 0.0f;
        float last_b_density_ = 0.0f;
        
        // Subscription to the camera topic
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

        // Callback for the camera
        void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);        

        // Publisher of the green density for finishing
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_pub_;

        // Publisher of only the center green density for tracking
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr center_green_density_pub_;
};

#endif  