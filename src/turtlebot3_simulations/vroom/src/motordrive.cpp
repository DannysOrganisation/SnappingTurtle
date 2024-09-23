/*
motordrive.cpp IMPLEMENTATION

This is the implementation for the motordrive node which reads from the
state topic and publishes to cmd_vel to actually control the robot

Written: Adam Riesel
Edited: Daniel Monteiro
*/

#include "motordrive.hpp"

Motordrive::Motordrive()
    : Node ("tb3_motor_drive")
{
    //initialise publisher to /cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // initialise subscriber to /state
    motor_driver_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "state", \
        10, \
        std::bind(
        &Motordrive::state_callback, \
        this, \
        std::placeholders::_1)
    );
}

Motordrive::~Motordrive()
{
    RCLCPP_INFO(this->get_logger(), "Motor_drive node has been terminated");
}


// Callback Function for motordriver subscriber
void Motordrive::state_callback(const std_msgs::msg::Int32 msg){
    
    //store current state
    current_state_ = msg.data;

    // determine which drive method to call
    switch (current_state_){

        case LOCATE_WALL:
            turn_right();
            break;

        case ROTATE_IN_PLACE:
            turn_right_fast();
            break;

        case TURN_TO_WALL:
            turn_right();
            break;

        case TB3_DRIVE_FORWARD:
            drive_forward();
            break;

        case TB3_SLOW_FORWARD:
            slow_forward();
            break;
        
        case TB3_LEFT_TURN:
            turn_left();
            break;
        
        case TB3_RIGHT_TURN:
            turn_right();
            break;
        
        case TB3_LEFT_TURN_90_DEG:
            turn_hard_left();
            break;

        case TB3_RIGHT_TURN_90_DEG:
            turn_hard_right();
            break;

        case STOP:
            stop();
            break;

        // Drive Forward as the default state
        default:
            drive_forward();
            break;

    }
}

// Publisher callback for the cmd_vel
void Motordrive::update_cmd_vel(double linear, double angular)
{
    //convert velocities into ros2 format for cmd_vel
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    //publish cmd_vel to /cmd_vel
    cmd_vel_pub_->publish(cmd_vel);
}

void Motordrive::turn_left(){
    update_cmd_vel(0.0, MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_left_slow()
{
    update_cmd_vel(0.0, MotorControl::SLOW_SCALING * MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_right(){
    update_cmd_vel(0.0, -1*MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_right_slow()
{
    update_cmd_vel(0.0, -1* MotorControl::SLOW_SCALING * MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_right_fast()
{
    update_cmd_vel(0.0, -1 * MotorControl::SPEED_SCALING * MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_hard_left()
{
    update_cmd_vel(MotorControl::SLOW_SCALING * 
                   MotorControl::SLOW_SCALING *
                   MotorControl::LINEAR_VELOCITY, MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_hard_right(){
    update_cmd_vel(0.0, -1*MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::drive_forward(){
    update_cmd_vel(MotorControl::LINEAR_VELOCITY, 0.0);
}

void Motordrive::stop()
{
    update_cmd_vel(0.0, 0.0);
}

void Motordrive::dance()
{
    update_cmd_vel(0.0, MotorControl::SPEED_SCALING *
    MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::slow_forward(){
    update_cmd_vel(MotorControl::SLOW_SCALING * MotorControl::LINEAR_VELOCITY, 0.0);
}

// Main function to spawn the node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Motordrive>());
  rclcpp::shutdown();
}