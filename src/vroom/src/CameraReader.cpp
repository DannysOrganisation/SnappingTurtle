/*
CameraReader.cpp

This is a file that defines the class for the node 
for the Camera Reader. This subscribes to the Image
topic and processes the data.

By James Hocking
*/


#include "CameraReader.hpp"


// constructor for the CameraReader class
CameraReader::CameraReader() : Node("turtlebot3_camera_reader") {
    // Subscribe to the camera topic
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",  // Corrected topic name
        rclcpp::SensorDataQoS(), 
        std::bind(&CameraReader::camera_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 CameraReader node has been initialised.");
}

// Destructor for the CameraReader class
CameraReader::~CameraReader() {
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 CameraReader node has been destroyed.");
}

// Callback function for the camera topic
void CameraReader::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Access the raw image data
    const auto &data = msg->data;

    // Initialize color counters
    float amount_of_r = 0;
    float amount_of_g = 0;
    float amount_of_b = 0;

    // Process the image data - structured as [r1, g1, b1, r2, g2, b2 ...]
    for (size_t i = 0; i < data.size(); i += 3) {
        // get each of the rgb values
        uint8_t r = data[i];       // Red channel
        uint8_t g = data[i + 1];   // Green channel
        uint8_t b = data[i + 2];   // Blue channel

        amount_of_r += r;
        amount_of_g += g;
        amount_of_b += b;
    }

    float total_amount_of_pixels = amount_of_r + amount_of_g + amount_of_b;
    
    // Calculate color densities as percentages, set member variables
    last_r_density_ = 100.0f * amount_of_r / total;
    last_g_density_ = 100.0f * amount_of_g / total;
    last_b_density_ = 100.0f * amount_of_b / total;
}

// Getter function for last red density recorded
float CameraReader::get_last_r_density() {
    return last_r_density_;
}

// Getter function for last green density recorded
float CameraReader::get_last_g_density() {
    return last_g_density_;
}

// Getter function for last blue density recorded
float CameraReader::get_last_b_density() {
    return last_b_density_;
}
