/*

FSM IMPLEMENTATION FILE

*/

#include "constants.hpp"
#include "lidar.hpp"
#include "odom.hpp" 
// #include "CameraReader.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class FSM : public rclcpp::Node
{

    public:
        FSM();
        ~FSM();


    private:
        // ROS topic publishers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;

        // callback function 
        void update_state();

        // timer to control how often state gets published
        rclcpp::TimerBase::SharedPtr update_timer_;


        // ROS topic subscribers
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr scan_data_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr density_sub_;

        void scan_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void robot_pose_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void density_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // the current state of the system
        int current_state_;

        // wall finding member variables
        double start_pose_;
        double min_distance_pose_;
        double min_distance_;
        

        // track poses that need to be remembe
        double robot_pose_;
        double prev_robot_pose_;

        // track the scan data that needs to be used
        std::vector<double> scan_data_;
        std::vector<double> prev_scan_data_;
        std::vector<double> temp_scan_data_;

        // track the amount of green that has been located
        double density_;
        double previous_density_;
        double max_density_;

        // a ros clck
        rclcpp::Clock ros_clk;
        rclcpp::Time current_time;
        bool locate_flag_;
        bool goal_detected_;
    
        // State Transition Logic
        void GET_TB3_DIRECTION_logic();
        void ROTATE_IN_PLACE_logic();
        void LOCATE_WALL_logic();

        // goal state transition_logic
        void DETECTED_GOAL_logic();

        void FIND_GOAL_RIGHT_logic();
        void FIND_GOAL_LEFT_logic();

        void TURN_TO_GOAL_logic();
        void DRIVE_TO_GOAL_logic();
};