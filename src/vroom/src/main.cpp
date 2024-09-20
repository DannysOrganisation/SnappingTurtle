#include "fsm.hpp"
#include "motordrive.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create the FSM state transition node (NOT FOR TESTING)
    rclcpp::spin(std::make_shared<FSM>());

    // Create the State Output Node (NOT FOR TESTING)
    rclcpp::spin(std::make_shared<Motordrive>());

    rclcpp::shutdown();

  return 0;
}
