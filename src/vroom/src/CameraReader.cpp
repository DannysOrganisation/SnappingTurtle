/*
CameraReader.cpp

This is a file that defines the class for the node 
for the Camera Reader. This subscribes to the Image
topic and processes the data.

By James Hocking
*/


#include "CameraReader.hpp"
#include "enum.h"


// constructor for the CameraReader class
CameraReader::CameraReader() : Node("turtlebot3_camera_reader") {
    // Subscribe to the camera topic
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",  // Corrected topic name
        rclcpp::SensorDataQoS(), 
        std::bind(&CameraReader::camera_callback, this, std::placeholders::_1)
    );

    // Create Quality of Service for publisher 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Create the publisher for the density of green in the image
    state_pub_ = this->create_publisher<std_msgs::msg::Float32>("green_density_topic", qos);

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
    for (size_t i = 0; i < data.size(); i += AMOUNT_OF_COLOURS) {
        // get each of the rgb values
        uint8_t r = data[i + RED_INDEX_ADJUSTMENT];       
        uint8_t g = data[i + GREEN_INDEX_ADJUSTMENT];   
        uint8_t b = data[i + BLUE_INDEX_ADJUSTMENT];   

        amount_of_r += r;
        amount_of_g += g;
        amount_of_b += b;
    }

    float total_amount_of_pixels = amount_of_r + amount_of_g + amount_of_b;
    
    // Calculate color densities as percentages, set member variables
    last_r_density_ = 100.0f * amount_of_r / total_amount_of_pixels;
    last_g_density_ = 100.0f * amount_of_g / total_amount_of_pixels;
    last_b_density_ = 100.0f * amount_of_b / total_amount_of_pixels;

    // publish the density of the green
    std_msgs::msg::Float32 density_msg;
    density_msg.data = last_g_density_;  // Assign float value to the message
    state_pub_->publish(density_msg);
}

// Getter function for last red density recorded
float CameraReader::get_last_r_density() const {
    return last_r_density_;
}

// Getter function for last green density recorded
float CameraReader::get_last_g_density() const {
    return last_g_density_;
}

// Getter function for last blue density recorded
float CameraReader::get_last_b_density() const {
    return last_b_density_;
}


// Main function to create the node
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<CameraReader>();

    // Spin the node to process callbacks
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}