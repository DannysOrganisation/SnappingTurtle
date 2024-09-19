#include <CameraReader.hpp>

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