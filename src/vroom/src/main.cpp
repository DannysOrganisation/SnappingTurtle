#include <CameraReader.hpp>
// ADD INCLUDES HERE ONCE WE'VE DECIDED ON A FINAL CLASS TO RUN EVERYTHING THROUGH

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