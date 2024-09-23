/*

FSM CLASS IMPLEMENTATION

This class has all the next state logic for the turtlbot as a state machine
It decides which is the next state, and publishes the current state to /state

Written: Daniel Monteiro
Editted: Adam Riesel

*/

#include "fsm.hpp"
#include <map>

using namespace std::chrono_literals;

//--
FSM::FSM(): Node("fsm_node"), current_state_(LOCATE_WALL)
{
    RCLCPP_INFO(this->get_logger(), "Wall to Follow: %d", wall_choice_);

    // set default values for all member variables
    scan_data_.resize(LidarAngles::NUM_ANGLES, 0.0);
    prev_scan_data_.resize(LidarAngles::NUM_ANGLES, 0.0);
    for(int i = 0; i < LidarAngles::NUM_ANGLES; i++)
    {
        prev_scan_data_[i] = Distance::MAX_DISTANCE;
        scan_data_[i] = Distance::MAX_DISTANCE;
    }    
    start_pose_ = 0.0;
    robot_pose_ = 0.0;
    prev_robot_pose_ = 0.0;

    density_ = 0.0;
    previous_density_ = 0.0;
    max_density_ = 0.0;

    // initialise the clock
    rclcpp::Clock ros_clock(RCL_SYSTEM_TIME); 

    //initialise LiDar subscriber
    scan_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "lidar", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::scan_data_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );
    
    // initialise robot pose subscriber
    robot_pose_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "robotpose", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::robot_pose_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

    // initialise density subscriber
    density_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "green_density", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::density_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );
    
    
    // initialise the state publisher   
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>("state", STANDARD_BUFFER_SIZE);

    // initialise the timer that will cotrol how often the state gets published
    update_timer_ = this->create_wall_timer(
            30ms, std::bind(&FSM::update_state, this));

    // display successful creation message
    RCLCPP_INFO(this->get_logger(), "FSM_node has been successfully initialised");
}

//--
FSM::~FSM()
{
    RCLCPP_INFO(this->get_logger(), "FSM node has been terminated");
}

//--
void FSM::density_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    //store message data into member variable
    density_ = msg->data;
    return;
}

//--
void FSM::scan_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    //check message validity
    if (msg->data.size() != LidarAngles::NUM_ANGLES) {
        RCLCPP_ERROR(this->get_logger(), "Received unexpected scan data size");
        return;
    }

    // Clear the existing data in scan_data_
    scan_data_.clear();
    scan_data_.resize(LidarAngles::NUM_ANGLES, 0.0);

    // Copy the data from the message to scan_data_
    scan_data_.assign(msg->data.begin(), msg->data.end());
    return;
}

//--
void FSM::robot_pose_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    //store message data into member variable
    robot_pose_ = msg->data;
    return;
}

//--
void FSM::update_state()
{
    // Create an integer message
    auto msg = std_msgs::msg::Int32();
    msg.data = current_state_;

    // Publish the message
    state_pub_->publish(msg);

    // store into temporary variable that won't change during processing
    temp_scan_data_ = scan_data_;

    //check if goal has been reached
    if(density_ > GoalTracking::GOAL_FOUND && prev_scan_data_[CENTER] < 0.5)
    {
        current_state_ = STOP;
        return;
    }

    // determine and switch to next state after publishing the current state
    switch (current_state_)
    {

        case LOCATE_WALL:
            LOCATE_WALL_logic();
            break;

        case ROTATE_IN_PLACE:
            ROTATE_IN_PLACE_logic();
            break;

        case TURN_TO_WALL:
            if(fabs(robot_pose_ - min_distance_pose_) < 5e-3)
                current_state_ = GET_TB3_DIRECTION;
            break;

        case GET_TB3_DIRECTION:
            GET_TB3_DIRECTION_logic();
            break;
        
        case TB3_DRIVE_FORWARD:

            // the velocity should be set in the output and then 
            // go back to watching for incoming objects
            current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_RIGHT_TURN:

            // stay in rotate state until robot reaches specified 
            // change in pose
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE)
            {
                current_state_ = GET_TB3_DIRECTION;
            }
            break;
        
        case TB3_LEFT_TURN:

            // stay in rotate state until robot reaches specified 
            // change in pose
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE)
            {
                current_state_ = GET_TB3_DIRECTION;
            }
            break;
        
        case TB3_LEFT_TURN_90_DEG:

            // stay in rotate state until robot reaches specified 
            // change in pose of 90 degrees
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE_90)
            {
                if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::CHECK_ANGLE_WRAP) {
                    prev_robot_pose_ -= 360 * DEG2RAD;
                }
                else {
                    current_state_ = GET_TB3_DIRECTION;
                }
            }
            break;

        case TB3_RIGHT_TURN_90_DEG:

            // stay in rotate state until robot reaches specified 
            // change in pose of 90 degrees    
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE_90)
            {
                if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::CHECK_ANGLE_WRAP) {
                    prev_robot_pose_ += 360 * DEG2RAD;
                }
                else {
                    current_state_ = GET_TB3_DIRECTION;
                }
            }
            break;
        
        case TB3_SLOW_FORWARD:

            // go straight to analysing direction after publishing
            // the slow forward current state
            current_state_ = GET_TB3_DIRECTION;
            break;
        default:
            current_state_ = GET_TB3_DIRECTION;
            break;
   }
}


//--
void FSM::GET_TB3_DIRECTION_logic()
{
    //if a wall is far away in front
    if (temp_scan_data_[CENTER] > Distance::CHECK_FORWARD_DIST)
    {   
        //if a wall is far away on the left
        if (temp_scan_data_[LEFT] < Distance::CHECK_SIDE_DIST)
        {
            //update previous data
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;

            //determine how to act for left or right wall follow
            if (wall_choice_ == WallFollowChoice::LEFT_WALL){
                current_state_ = TB3_RIGHT_TURN;
            }
            else if (wall_choice_ == WallFollowChoice::RIGHT_WALL){
                current_state_ = TB3_LEFT_TURN;
            }
        }

        // if wall on right is too close
        else if (temp_scan_data_[RIGHT] < Distance::CHECK_SIDE_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            if (wall_choice_ == WallFollowChoice::LEFT_WALL){
                current_state_ = TB3_LEFT_TURN;
            }
            else if (wall_choice_ == WallFollowChoice::RIGHT_WALL){
                current_state_ = TB3_RIGHT_TURN;
            }
        }

        // if a left wall suddenly disappeared
        else if (temp_scan_data_[HARD_LEFT] > (Distance::NO_WALL_DIST) && prev_scan_data_[HARD_LEFT] <= Distance::NO_WALL_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_LEFT_TURN_90_DEG;
        }

        //if a right wall suddenly disappears
        else if (temp_scan_data_[HARD_RIGHT] > (Distance::NO_WALL_DIST) && prev_scan_data_[HARD_RIGHT] <= Distance::NO_WALL_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_RIGHT_TURN_90_DEG;
        }

        // if the wall on the left is getting  further away, turn towards it
        // if doing left wall follow
        else if (wall_choice_ == WallFollowChoice::LEFT_WALL && temp_scan_data_[LEFT] > prev_scan_data_[LEFT] && temp_scan_data_[LEFT] >  (1.3 * Distance::CHECK_SIDE_DIST) && (temp_scan_data_[LEFT] < 2* Distance::CHECK_SIDE_DIST)){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_LEFT_TURN;
        }

        // if the wall on the left is getting  further away, turn towards it
        // if doing right wall follow
        else if (wall_choice_ == WallFollowChoice::RIGHT_WALL && temp_scan_data_[RIGHT] > prev_scan_data_[RIGHT] && temp_scan_data_[RIGHT] >  (1.3 * Distance::CHECK_SIDE_DIST) && (temp_scan_data_[RIGHT] < 2* Distance::CHECK_SIDE_DIST)){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_RIGHT_TURN;
        }

        //if a left hand corner is approaching slow down during left wall follow
        else if (wall_choice_ == WallFollowChoice::LEFT_WALL && temp_scan_data_[LEFT] > 1.5 * Distance::CHECK_SIDE_DIST && prev_scan_data_[LEFT] > 1.5 * Distance::CHECK_SIDE_DIST){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_SLOW_FORWARD;
        }

        //if right corner is approaching during right wall follow
        else if (wall_choice_ == WallFollowChoice::RIGHT_WALL && temp_scan_data_[RIGHT] > 1.5 * Distance::CHECK_SIDE_DIST && prev_scan_data_[RIGHT] > 1.5 * Distance::CHECK_SIDE_DIST){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_SLOW_FORWARD;
        }

        // if we are not close to any walls then keep driving forward
        else {
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_DRIVE_FORWARD;
        }
    }

    // if there is something in front of the robot then turn right
    else if (wall_choice_ == WallFollowChoice::LEFT_WALL && temp_scan_data_[CENTER] < Distance::CHECK_FORWARD_DIST) {
        prev_scan_data_ = temp_scan_data_;
        prev_robot_pose_ = robot_pose_;
        current_state_ = TB3_RIGHT_TURN_90_DEG;
    }
    else if (wall_choice_ == WallFollowChoice::RIGHT_WALL && temp_scan_data_[CENTER] < Distance::CHECK_FORWARD_DIST) {
        prev_scan_data_ = temp_scan_data_;
        prev_robot_pose_ = robot_pose_;
        current_state_ = TB3_RIGHT_TURN_90_DEG;
    }
}


void FSM::ROTATE_IN_PLACE_logic()
{

    if(ros_clk.now().seconds() - current_time.seconds() > MotorControl::TIME_FOR_HALF_ROTATION)
        locate_flag_ = true;

    // Check that we are back to the starting position 
    //(we've done a full rotation)
    if(locate_flag_ && fabs(robot_pose_ - start_pose_) < 1e-1)
        current_state_ = TURN_TO_WALL;
    
    // as we rotate check to see if we've found a closer wall yet 
    // handle bad data returning 0
    if(temp_scan_data_[CENTER] < min_distance_ && temp_scan_data_[CENTER] > 1e-2)
    {
        min_distance_ = temp_scan_data_[CENTER];
        min_distance_pose_ = robot_pose_;
    }
}

void FSM::LOCATE_WALL_logic()
{
    RCLCPP_INFO(this->get_logger(), "Attempting to find the closest wall");

    // set relevant variables for beggining wall search
    start_pose_ = robot_pose_;
    min_distance_ = Distance::MAX_DISTANCE;
    current_state_ = ROTATE_IN_PLACE;
    locate_flag_ = false;

    // set the timer
    current_time = ros_clk.now();
}

#ifdef FSM_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FSM>());
  rclcpp::shutdown();
}
#endif