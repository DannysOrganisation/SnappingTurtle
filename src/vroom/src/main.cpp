#include "fsm.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create the FSM state transition node (NOT FOR TESTING)
    rclcpp::spin(std::make_shared<FSM>());

    rclcpp::shutdown();

  return 0;
}
