/*
Odometry node IMPLEMENTATION
*/

#include "vroom/odom.hpp"

using namespace std::chrono_literals;

Odom::Odom() : Node("Odometry_Node")
{   

    // initialise the current pose
    robot_pose_ = 0.0
    prev_robot_pose_ = 0.0;

    // Initialise subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

    // display successful creation message
    RCLCPP_INFO(this->get_logger(), "Odometry_Node has been successfully initialised");
}

Odom::~Odom()
{
  RCLCPP_INFO(this->get_logger(), "Odometry_Node has been terminated");
}


/*
The following function takes in the robots odometry and outputs stores the
yaw direction of the robot in 'robot_pose' 

@param msg: a nav_msgs::msg::Odometry::SharedPtr that corresponds to a quaternion (4 dimensional)

@return yaw: The current yaw angle in radians of the robot.
*/
void Odom::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // break the message down into a quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  // do the math operation to convert the pose into a 3x3 matrix
  // for the orientation
  tf2::Matrix3x3 m(q);

  // extract the angles from the data
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // set the member variable to the yaw (robot should only have rotation in this dimension)
  robot_pose_ = yaw;
}


/*
Getter for the robot pose. 
Const used such that the getter can't change the robot
pose itself.
*/

double get_robot_pose() const
{
    return robot_pose_;
}


