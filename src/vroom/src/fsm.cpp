/*

FSM CLASS IMPLEMENTATION

*/

#include "vroom/fsm.hpp"

using namespace std::chrono_literals;

FSM::FSM(): Node("fsm_node"), current_state_(GET_TB3_DIRECTION)
{
    // Create instances of Lidar, Odom, and Cam nodes
    lidar_node_ = std::make_shared<Lidar>();
    odom_node_ = std::make_shared<Odom>();
    cam_node_ = std::make_shared<Cam>();

    // create the state publisher
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/state", STANDARD_BUFFER_SIZE);

    // create the timer that will cotrol how often the state gets published
    update_timer_ = this->create_wall_timer(
            10ms, std::bind(&FSM::update_state, this));
}

FSM::~FSM()
{
    RCLCPP_INFO(this->get_logger(), "FSM node has been terminated");
}


void FSM::update_state()
{

    // Create an integer message
    auto msg = std_msgs::msg::Int32();
    msg.data = current_state_;

    // Publish the message
    state_pub_->publish(msg);

    // Log the current state
    RCLCPP_INFO(this->get_logger(), "Published state: %d", msg.data);


    /*
    TO DO

    WORK OUT THE NEXT STATE LOGIC AND PROCESS IT HERE
    */

   switch (current_state_)
   {

        case LOCATE_WALL:
            /*
            STILL HAVE TO DO THIS
            */
            break;

        case TURN_TO_WALL:
            /*
            STILL HAVE TO IMPLEMENT THIS
            */
            break;

        case GET_TB3_DIRECTION:

            // extract the lidar data
            scan_data_ = lidar_node_.get_scan_data();
    
            if (scan_data_[CENTER] > CHECK_FORWARD_DIST)
            {
                if (scan_data_[LEFT] < CHECK_SIDE_DIST)
                {
                    prev_robot_pose_ = odom_node_.get_robot_pose();
                    prev_scan_data_ = scan_data_;
                    current_state_ = TB3_RIGHT_TURN;
                }
                else if (scan_data_[RIGHT] < CHECK_SIDE_DIST)
                {
                    prev_robot_pose_ = odom_node_.get_robot_pose();
                    prev_scan_data_ = scan_data_;
                    current_state_ = TB3_LEFT_TURN;
                }
                else if (scan_data_[HARD_LEFT] > (NO_WALL_DIST) && prev_scan_data_[HARD_LEFT] <= NO_WALL_DIST)
                {
                    prev_robot_pose_ = robot_pose_;
                    prev_scan_data_ = scan_data_;
                    turtlebot3_state_num = TB3_LEFT_90;
                }
                // if the wall on the left is getting too further away, turn towards it
                else if (scan_data_[LEFT] > prev_scan_data_[LEFT] && scan_data_[LEFT] >  (1.3 * CHECK_SIDE_DIST) && (scan_data_[LEFT] < 2* CHECK_SIDE_DIST)){
                prev_robot_pose_ = robot_pose_;
                prev_scan_data_ = scan_data_;
                turtlebot3_state_num = TB3_LEFT_TURN;
                }
                //if a left hand corner is approaching
                else if (scan_data_[LEFT] > 1.5 * CHECK_SIDE_DIST && prev_scan_data_[LEFT] > 1.5 * CHECK_SIDE_DIST){
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
            if (scan_data_[CENTER] < CHECK_FORWARD_DIST) {
                prev_scan_data_ = scan_data_;
                prev_robot_pose_ = robot_pose_;
                turtlebot3_state_num = TB3_RIGHT_TURN;
            }

            break;
        
        case TB3_DRIVE_FOWARD:

            // the velocity should be set in the output and then 
            // go back to watching for incoming objects
            current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_RIGHT_TURN:
            
            // stay in rotate state until we get to a better position
            if fabs(prev_robot_pose_ - odom.get_robot_pose()) >= escape_range
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_LEFT_TURN:
            // stay in rotate state until we get to a better position
            if fabs(prev_robot_pose_ - odom.get_robot_pose()) >= escape_range
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_LEFT_90:
             if (fabs(prev_robot_pose_ - robot_pose_) >= escape_90)
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_SLOW_FORWARD:
            // go straight to analysing direction after publishing the slow forward
            current_state_ = GET_TB3_DIRECTION;
            break;
   }


   switch (current_state_)
   {

        case LOCATE_WALL:
            
            break;

        case GET_TB3_DIRECTION:

            //TODO ADD IN THIS BASED ON WHATEVER ADAM ENDS UP DOING WITH THE LIDAR
            scan_reference;
            break;
        
        case TB3_DRIVE_FOWARD:

            // the velocity should be set in the output and then 
            // go back to watching for incoming objects
            current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_RIGHT_TURN:
            
            // stay in rotate state until we get to a better position
            if fabs(prev_robot_pose_ - odom.get_robot_pose()) >= escape_range
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_LEFT_TURN:
            // stay in rotate state until we get to a better position
            if fabs(prev_robot_pose_ - odom.get_robot_pose()) >= escape_range
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_LEFT_90:
             if (fabs(prev_robot_pose_ - robot_pose_) >= escape_90)
                current_state_ = GET_TB3_DIRECTION;
            break;
        
        case TB3_SLOW_FORWARD:
            // go straight to analysing direction after publishing the slow forward
            current_state_ = GET_TB3_DIRECTION;
            break;
   }
}