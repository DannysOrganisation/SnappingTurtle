// ADD INCLUDES HERE ONCE WE'VE DECIDED ON A FINAL CLASS TO RUN EVERYTHING THROUGH
#include "fsm.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FSM>());
    rclcpp::shutdown();

  return 0;
}