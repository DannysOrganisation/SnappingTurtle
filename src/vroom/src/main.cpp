#include "lidar.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CLidar>()); //spin LiDar
    rclcpp::shutdown();

  return 0;
}
