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

void Motordrive::state_callback(const std_msgs::msg::Int32 msg){
    
    //store current state
    current_state_ = msg.data;

    // determine which drive method to call
    switch (current_state_){

        case LOCATE_WALL:
            turn_right();
            break;

        case ROTATE_IN_PLACE:
            turn_right();
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
        
        //added default state to drive forward... probably not the best option
        //I think a better option would be to come to a stop? not sure tbh
        default:
            drive_forward();
            break;

    }
}

void Motordrive::update_cmd_vel(double linear, double angular)
{

    //convert velocities into ros2 format for cmd_vel
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    //publish cmd_vel to /cmd_vel
    cmd_vel_pub_->publish(cmd_vel);
}

//update cmd_vel appropriately for each method
void Motordrive::turn_left(){
    update_cmd_vel(0.0, MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_right(){
    update_cmd_vel(0.0, -1*MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_hard_left(){
    update_cmd_vel(0.1*MotorControl::LINEAR_VELOCITY, MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::turn_hard_right(){
    update_cmd_vel(0.1*MotorControl::LINEAR_VELOCITY,-1*MotorControl::ANGULAR_VELOCITY);
}

void Motordrive::drive_forward(){
    update_cmd_vel(MotorControl::LINEAR_VELOCITY, 0.0);
}

void Motordrive::stop()
{
    update_cmd_vel(0.0, 0.0);
}

void Motordrive::slow_forward(){
    update_cmd_vel(0.5 * MotorControl::LINEAR_VELOCITY, 0.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Motordrive>());
  rclcpp::shutdown();
}