/*

FSM CLASS IMPLEMENTATION

*/

#include "fsm.hpp"

using namespace std::chrono_literals;

FSM::FSM(): Node("fsm_node"), current_state_(GET_TB3_DIRECTION)
{
    
    //create LiDar subscriber
    scan_data_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "lidar", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::scan_data_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );
    
    // create robot pose subscriber
    robot_pose_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "robotpose", /* subscribe to topic /lidar */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &FSM::robot_pose_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );
    
    
    // create the state publisher   
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>("state", STANDARD_BUFFER_SIZE);

    // create the timer that will cotrol how often the state gets published
    update_timer_ = this->create_wall_timer(
            10ms, std::bind(&FSM::update_state, this));

    // display successful creation message
    RCLCPP_INFO(this->get_logger(), "FSM_node has been successfully initialised");
}

FSM::~FSM()
{
    RCLCPP_INFO(this->get_logger(), "FSM node has been terminated");
}

void FSM::scan_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
// Clear the existing data in scan_data_
    scan_data_.clear();

    // Copy the data from the message to scan_data_
    scan_data_.insert(scan_data_.end(), msg->data.begin(), msg->data.end());
    return;
}

void FSM::robot_pose_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    robot_pose_ = msg->data;
    return;
}


void FSM::update_state()
{

    // Create an integer message
    auto msg = std_msgs::msg::Int32();
    msg.data = current_state_;

    // Publish the message
    state_pub_->publish(msg);

    // Log the current state
    // RCLCPP_INFO(this->get_logger(), "Published state: %d", msg.data);


    /*
    TO DO

    WORK OUT THE NEXT STATE LOGIC AND PROCESS IT HERE
    */

   switch (current_state_)
   {

        case LOCATE_WALL:
            
            start_pose_ = robot_pose_;
            break;

        case ROTATE_IN_PLACE:

            if(fabs(robot_pose_ - start_pose_ )< 0.01)
                current_state_ = TURN_TO_WALL;
            break;

        case TURN_TO_WALL:
            /*
            STILL HAVE TO IMPLEMENT THIS
            */
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
            
            // stay in rotate state until we get to a better position
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE)
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_LEFT_TURN:
            // stay in rotate state until we get to a better position
            if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE)
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_LEFT_TURN_90_DEG:
             if (fabs(prev_robot_pose_ - robot_pose_) >= Distance::ESCAPE_RANGE_90)
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_SLOW_FORWARD:
            // go straight to analysing direction after publishing the slow forward
            current_state_ = GET_TB3_DIRECTION;
            break;
   }
}


/*
Next State Logic For Basic Direction Getting Function
*/
void FSM::GET_TB3_DIRECTION_logic()
{
    if (scan_data_[CENTER] > Distance::CHECK_FORWARD_DIST)
    {   
        if (scan_data_[LEFT] < Distance::CHECK_SIDE_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = scan_data_;
            current_state_ = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < Distance::CHECK_SIDE_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = scan_data_;
            current_state_ = TB3_LEFT_TURN;
        }
        else if (scan_data_[HARD_LEFT] > (Distance::NO_WALL_DIST) && prev_scan_data_[HARD_LEFT] <= Distance::NO_WALL_DIST)
        {
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = scan_data_;
            current_state_ = TB3_LEFT_TURN_90_DEG;
        }
        // if the wall on the left is getting too further away, turn towards it
        else if (scan_data_[LEFT] > prev_scan_data_[LEFT] && scan_data_[LEFT] >  (1.3 * Distance::CHECK_SIDE_DIST) && (scan_data_[LEFT] < 2* Distance::CHECK_SIDE_DIST)){
        prev_robot_pose_ = robot_pose_;
        prev_scan_data_ = scan_data_;
        current_state_ = TB3_LEFT_TURN;
        }
        //if a left hand corner is approaching
        else if (scan_data_[LEFT] > 1.5 * Distance::CHECK_SIDE_DIST && prev_scan_data_[LEFT] > 1.5 * Distance::CHECK_SIDE_DIST){
            prev_robot_pose_ = robot_pose_;
            prev_scan_data_ = scan_data_;
            current_state_ = TB3_SLOW_FORWARD;
        }
        // if we are not close to any walls then keep driving forward
        else {
            prev_scan_data_ = scan_data_;
            current_state_ = TB3_DRIVE_FORWARD;
        }
    }

    // if there is something in front of the robot then turn right
    if (scan_data_[CENTER] < Distance::CHECK_FORWARD_DIST) {
        prev_scan_data_ = scan_data_;
        prev_robot_pose_ = robot_pose_;
        current_state_ = TB3_RIGHT_TURN;
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