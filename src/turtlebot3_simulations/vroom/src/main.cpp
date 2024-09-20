// Use this file for testing individual modules
// Use snapping_turtle_launch.py for doing system level tests
#include "fsm.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FSM>());
    rclcpp::shutdown();

  return 0;
}