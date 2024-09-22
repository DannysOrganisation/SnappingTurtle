/*

FSM CLASS IMPLEMENTATION

*/

#include "fsm.hpp"
#include <map>

using namespace std::chrono_literals;

// FSM::FSM(): Node("fsm_node"), current_state_(LOCATE_WALL)
FSM::FSM(): Node("fsm_node"), current_state_(LOCATE_WALL)
{
    RCLCPP_INFO(this->get_logger(), "Wall to Follow: %d", wall_choice_);
    // set default values for everything
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

    goal_detected_ = false;

    // initialise the clock
    rclcpp::Clock ros_clock(RCL_SYSTEM_TIME); 


    //create LiDar subscriber
    scan_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "lidar", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::scan_data_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );
    
    // create robot pose subscriber
    robot_pose_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "robotpose", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::robot_pose_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

    // density subscriber
    density_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "green_density", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::density_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );
    
    
    // create the state publisher   
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>("state", STANDARD_BUFFER_SIZE);

    // create the timer that will cotrol how often the state gets published
    update_timer_ = this->create_wall_timer(
            30ms, std::bind(&FSM::update_state, this));

    // display successful creation message
    RCLCPP_INFO(this->get_logger(), "FSM_node has been successfully initialised");
}

FSM::~FSM()
{
    RCLCPP_INFO(this->get_logger(), "FSM node has been terminated");
}

void FSM::density_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    density_ = msg->data;
    return;
}

void FSM::scan_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
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

void FSM::robot_pose_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    robot_pose_ = msg->data;
    return;
}


void FSM::update_state()
{

    // Create an integer message
    auto msg = std_msgs::msg::Int32();
    msg.data = current_state_;


    // RCLCPP_INFO(this->get_logger(), ("Current State: " + m.at(current_state_)).c_str());

    // Publish the message
    state_pub_->publish(msg);

    // create a temporary variable that won't change during processing
    temp_scan_data_ = scan_data_;

    // determine if we enter the goal seeking states
    // if(density_ > GoalTracking::GOAL_DETECT_LOWER_THRESHOLD && !(goal_detected_))
    // {
    //     goal_detected_ = true; 
    //     current_state_ = DETECTED_GOAL;
    //     return;
    // }
    if(density_ > GoalTracking::GOAL_FOUND && prev_scan_data_[CENTER] < 0.5)
    {
        current_state_ = STOP;
        return;
    }

   switch (current_state_)
   {

        case LOCATE_WALL:
            LOCATE_WALL_logic();
            break;

        case ROTATE_IN_PLACE:
            ROTATE_IN_PLACE_logic();
            break;

        case TURN_TO_WALL:
            // RCLCPP_INFO(this->get_logger(), "Closest wall found. Turning towards it. target_pose %f current pose %f", min_distance_pose_, robot_pose_);
            if(fabs(robot_pose_ - min_distance_pose_) < 5e-3)
                current_state_ = GET_TB3_DIRECTION;
            break;

        case GET_TB3_DIRECTION:

            // RCLCPP_INFO(this->get_logger(), "Scan Data: %f %f %f", scan_data_[0], scan_data_[1], scan_data_[2]);
            GET_TB3_DIRECTION_logic();
            break;
        
        case TB3_DRIVE_FORWARD:

            // the velocity should be set in the output and then 
            // go back to watching for incoming objects
            current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_RIGHT_TURN:
            // RCLCPP_INFO(this->get_logger(), "Turning right. old_pose %f current pose %f", prev_robot_pose_, robot_pose_);
            // stay in rotate state until we get to a better position
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE)
            {
                current_state_ = GET_TB3_DIRECTION;
            }
            break;
        
        case TB3_LEFT_TURN:
            // RCLCPP_INFO(this->get_logger(), "Turning left. old_pose %f current pose %f", prev_robot_pose_, robot_pose_);
            // stay in rotate state until we get to a better position
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE)
            {
                current_state_ = GET_TB3_DIRECTION;
            }
            break;
        
        case TB3_LEFT_TURN_90_DEG:
            // RCLCPP_INFO(this->get_logger(), "Turning left. old_pose %f current pose %f", prev_robot_pose_, robot_pose_);
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
            // RCLCPP_INFO(this->get_logger(), "Turning right. old_pose %f current pose %f", prev_robot_pose_, robot_pose_);
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
            // go straight to analysing direction after publishing the slow forward
            current_state_ = GET_TB3_DIRECTION;
            break;

        case DETECTED_GOAL:
            DETECTED_GOAL_logic();
            break;
        
        case FIND_GOAL_RIGHT:
            FIND_GOAL_RIGHT_logic();
            break; 
        
        case FIND_GOAL_LEFT: 
            FIND_GOAL_LEFT_logic();
            break;

        case FIND_GOAL_AVOID_WALL_LEFT:
            FIND_GOAL_AVOID_WALL_LEFT_logic();
            break;

        case FIND_GOAL_AVOID_WALL_RIGHT: 
            FIND_GOAL_AVOID_WALL_RIGHT_logic();
            break;
        
        case DRIVE_TO_GOAL: 
            DRIVE_TO_GOAL_logic();
            break;


   }
}


/*
Next State Logic For Basic Direction Getting Function
*/
void FSM::GET_TB3_DIRECTION_logic()
{
    if (temp_scan_data_[CENTER] > Distance::CHECK_FORWARD_DIST)
    {   
        if (temp_scan_data_[LEFT] < Distance::CHECK_SIDE_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            if (wall_choice_ == WallFollowChoice::LEFT_WALL){
                current_state_ = TB3_RIGHT_TURN;
            }
            else if (wall_choice_ == WallFollowChoice::RIGHT_WALL){
                current_state_ = TB3_LEFT_TURN;
            }
        }
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
        else if (temp_scan_data_[HARD_LEFT] > (Distance::NO_WALL_DIST) && prev_scan_data_[HARD_LEFT] <= Distance::NO_WALL_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_LEFT_TURN_90_DEG;
        }
        else if (temp_scan_data_[HARD_RIGHT] > (Distance::NO_WALL_DIST) && prev_scan_data_[HARD_RIGHT] <= Distance::NO_WALL_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_RIGHT_TURN_90_DEG;
        }
        // if the wall on the left is getting too further away, turn towards it
        else if (wall_choice_ == WallFollowChoice::LEFT_WALL && temp_scan_data_[LEFT] > prev_scan_data_[LEFT] && temp_scan_data_[LEFT] >  (1.3 * Distance::CHECK_SIDE_DIST) && (temp_scan_data_[LEFT] < 2* Distance::CHECK_SIDE_DIST)){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_LEFT_TURN;
        }
        else if (wall_choice_ == WallFollowChoice::RIGHT_WALL && temp_scan_data_[RIGHT] > prev_scan_data_[RIGHT] && temp_scan_data_[RIGHT] >  (1.3 * Distance::CHECK_SIDE_DIST) && (temp_scan_data_[RIGHT] < 2* Distance::CHECK_SIDE_DIST)){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_RIGHT_TURN;
        }
        //if a left hand corner is approaching
        else if (wall_choice_ == WallFollowChoice::LEFT_WALL && temp_scan_data_[LEFT] > 1.5 * Distance::CHECK_SIDE_DIST && prev_scan_data_[LEFT] > 1.5 * Distance::CHECK_SIDE_DIST){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = temp_scan_data_;
            current_state_ = TB3_SLOW_FORWARD;
        }
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
        // RCLCPP_INFO(this->get_logger(), "Time difference is %f", current_time.seconds() - ros_clk.now().seconds());
        // RCLCPP_INFO(this->get_logger(), "Locate flag has been reset");


    // Check that we are back to the starting position (we've done a full rotation)
    if(locate_flag_ && fabs(robot_pose_ - start_pose_) < 1e-1)
        current_state_ = TURN_TO_WALL;
    
    // as we rotate check to see if we've found a closer wall yet handle bad data returning 0
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

void FSM::DETECTED_GOAL_logic()
{
    previous_density_ = density_;
    current_state_ = FIND_GOAL_RIGHT;
}

void FSM::FIND_GOAL_RIGHT_logic()
{   

    // check if we're turning in the wrong direction
    if(density_ < previous_density_)
        current_state_ = FIND_GOAL_LEFT;
    else
    {
        current_state_ = DRIVE_TO_GOAL;
    }

    if (density_ > max_density_)
        max_density_ = density_;

    previous_density_ = density_;
    
}


void FSM::FIND_GOAL_LEFT_logic()
{


    // check if we're turning in the wrong direction
    if(density_ < previous_density_)
        current_state_ = FIND_GOAL_RIGHT;
    else
    {
        current_state_ = DRIVE_TO_GOAL;
    }

    if (density_ > max_density_)
        max_density_ = density_;
    
    previous_density_ = density_;
}


void FSM::FIND_GOAL_AVOID_WALL_LEFT_logic()
{
    if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE_90)
        current_state_ = DRIVE_TO_GOAL;
        return;
}


void FSM::FIND_GOAL_AVOID_WALL_RIGHT_logic()
{
    if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE_90)
        current_state_ = DRIVE_TO_GOAL;
        return;
}


void FSM::DRIVE_TO_GOAL_logic()
{   

    // check that we don't hit into anything
    if (temp_scan_data_[LEFT] < Distance::CHECK_SIDE_DIST)
    {
        prev_robot_pose_ = robot_pose_;
        prev_scan_data_ = temp_scan_data_;
        current_state_ = FIND_GOAL_AVOID_WALL_RIGHT;
        return;
    }
    else if (temp_scan_data_[RIGHT] < Distance::CHECK_SIDE_DIST)
    {
        prev_robot_pose_ = robot_pose_;
        prev_scan_data_ = temp_scan_data_;
        current_state_ = FIND_GOAL_AVOID_WALL_LEFT;
        return;
    }
    
    // check if we've headed in the wrong direction
    // if we've reached our destination then dance!
    if(density_ > GoalTracking::GOAL_FOUND)
    {
        current_state_ = DANCE;
    }
    else if(density_ < previous_density_)
    {
        current_state_ = DETECTED_GOAL;
    }
    

}

#ifdef FSM_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FSM>());
  rclcpp::shutdown();
}
#endif