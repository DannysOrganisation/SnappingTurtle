/*
CameraReader.cpp

This is a file that defines the class for the node 
for the Camera Reader. This subscribes to the Image
topic and processes the data.

By James Hocking
*/


#include "camera.hpp"


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
    state_pub_ = this->create_publisher<std_msgs::msg::Float32>("green_density", 
                                                                        qos);

    // Create the publisher for if the green is in the center of the image
    center_green_density_pub_ = this->create_publisher<std_msgs::msg::Bool>
                                                    ("is_green_center", qos);

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 CameraReader node has been 
                                                                initialised.");
}

// Destructor for the CameraReader class
CameraReader::~CameraReader() {
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 CameraReader node has been 
                                                                destroyed.");
}

// Callback function for the camera topic
void CameraReader::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Access the raw image data
    const auto &data = msg->data;

    // Image width and height
    int width = msg->width;
    int height = msg->height;

    // Start and end of Mask
    int starting_index = static_cast<int>(0.4 * width);
    int end_index = static_cast<int>(0.6 * width);
    int valid_rows = static_cast<int>(0.6 * height);

    // Initialize color counters
    float amount_of_r = 0;
    float amount_of_g = 0;
    float amount_of_b = 0;

    // Initialised center green
    float amount_of_green_center = 0;
    float amount_of_pixels_center = (end_index - starting_index) * valid_rows * 
                                                            AMOUNT_OF_COLOURS;

    // iterate through each row
    for (int row = 0; row < height; row++){
        // and then each pixel in that row
        for (int column = 0; column < width; column++){
            int actual_index = row * (width * AMOUNT_OF_COLOURS) + 
                                column * AMOUNT_OF_COLOURS;
            uint8_t r = data[actual_index + RED_INDEX_ADJUSTMENT];       
            uint8_t g = data[actual_index + GREEN_INDEX_ADJUSTMENT];   
            uint8_t b = data[actual_index + BLUE_INDEX_ADJUSTMENT];

            amount_of_r += r;
            amount_of_g += g;
            amount_of_b += b;

            if (column >= starting_index && column <= starting_index && 
                                            row < valid_rows) {
                amount_of_green_center += g;
            }
        }
    }

    float total_amount_of_pixels = amount_of_r + amount_of_g + amount_of_b;
    
    // Calculate color densities as percentages, set member variables
    last_r_density_ = 100.0f * amount_of_r / total_amount_of_pixels;
    last_g_density_ = 100.0f * amount_of_g / total_amount_of_pixels;
    last_b_density_ = 100.0f * amount_of_b / total_amount_of_pixels;

    // calculate center amount of green
    float center_percent_of_green = 100.0f * amount_of_green_center / 
                                            amount_of_pixels_center;
    bool is_looking_at_goal = center_percent_of_green > 
                                ColorThresholds::GREEN_CENTER_THESHOLD;

    // publish the density of the green
    std_msgs::msg::Float32 density_msg;
    density_msg.data = last_g_density_;  // Assign float value to the message
    state_pub_->publish(density_msg);


    // publish if looking at the green thing
    std_msgs::msg::Bool center_density_message;
    center_density_message.data = is_looking_at_goal;
    center_green_density_pub_->publish(center_density_message); 
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