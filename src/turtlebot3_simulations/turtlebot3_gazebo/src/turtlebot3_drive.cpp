// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <memory>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_.resize(4);
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;
  scan_data_[3] = 0.0;

  prev_scan_data_ = scan_data_;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // uint16_t scan_angle[9] = {0, 15, 345, 75, 90, 105, 285, 270, 255};
  uint16_t scan_angle[4] = {0, 30, 330, 90};

  for (int num = 0; num < 4; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      // double ave = (msg->ranges.at(scan_angle[3*num]) + (msg->ranges.at(scan_angle[3*num + 1])) + (msg->ranges.at(scan_angle[3*num + 2]))) / 3;
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  static uint8_t turtlebot3_state_num = 0;
  double escape_range = 2 * DEG2RAD;
  double escape_90 = 90 * DEG2RAD;
  double check_forward_dist = 0.4;
  double check_side_dist = 0.4;
  double no_wall_dist = 0.57;

  switch (turtlebot3_state_num) {

    // state of getting direction 
    case GET_TB3_DIRECTION:

      // check that there is not an object in front of the bot
      if (scan_data_[CENTER] > check_forward_dist) {

        // if there is a wall too close to the left then turn right
        if (scan_data_[LEFT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          prev_scan_data_ = scan_data_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        // if there is a wall too close to the right then turn left
        else if (scan_data_[RIGHT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          prev_scan_data_ = scan_data_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        //if the wall on the left suddenly drops away
        else if (scan_data_[HARD_LEFT] > (no_wall_dist) && prev_scan_data_[HARD_LEFT] <= no_wall_dist){
          prev_robot_pose_ = robot_pose_;
          prev_scan_data_ = scan_data_;
          turtlebot3_state_num = TB3_LEFT_90;
        }
        // if the wall on the left is getting too further away, turn towards it
        else if (scan_data_[LEFT] > prev_scan_data_[LEFT] && scan_data_[LEFT] >  (1.3 * check_side_dist) && (scan_data_[LEFT] < 2* check_side_dist)){//} || prev_scan_data_[LEFT] < 1.4 * check_side_dist)){
          prev_robot_pose_ = robot_pose_;
          prev_scan_data_ = scan_data_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        
        //if a eft hand corner is approaching
        else if (scan_data_[LEFT] > 1.5 * check_side_dist && prev_scan_data_[LEFT] > 1.5 * check_side_dist){
          prev_robot_pose_ = robot_pose_;
          prev_scan_data_ = scan_data_;
          turtlebot3_state_num = TB3_SLOW_FORWARD;
        }
        // if we are not close to any walls then keep driving forward
        else {
          prev_scan_data_ = scan_data_;
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      // if there is something in front of the robot then turn right
      if (scan_data_[CENTER] < check_forward_dist) {
        prev_scan_data_ = scan_data_;
        prev_robot_pose_ = robot_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    // driving forward state
    case TB3_DRIVE_FORWARD:

      // update the linear velocity for the robot. Keep the angular velocity 0
      update_cmd_vel(LINEAR_VELOCITY, 0.0);

      // go back to getting sensor information
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    // state for turning right
    case TB3_RIGHT_TURN:

      // determine if the robot has turned the full amount
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      }
      // if the current pose won't face the wall then rotate the turtlebot
      else {
        update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
      }
      break;

    // left turn same as right turn but in opposite direction
    case TB3_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.0, ANGULAR_VELOCITY);
      }
      break;

    case TB3_LEFT_90:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_90) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.1*LINEAR_VELOCITY, ANGULAR_VELOCITY);
      }
      break;

    case TB3_SLOW_FORWARD:

      // reduce speed
      update_cmd_vel(0.5 * LINEAR_VELOCITY, 0.0);

      // go back to getting sensor information
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    // default state gets the direction
    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
