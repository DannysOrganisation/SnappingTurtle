/*

FSM CLASS INTERFACE

This class has all the next state logic for the turtlbot as a state machine
It decides which is the next state, and publishes the current state to /state

Written: Daniel Monteiro
Editted: Adam Riesel

*/

#include "constants.hpp"
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

        /**
         * @brief this function publishes the state machine's current state to 
         * the prescribed topic. It then updates the next state and switches
         * states accordingly
         */
        void update_state();

        // timer to control how often state gets published
        rclcpp::TimerBase::SharedPtr update_timer_;


        // ROS topic subscribers
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr \
        scan_data_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr density_sub_;

        /**
         * @brief this function is called each time a message is received via
         * the topic: /lidar. It updates scan_data_ member variable
         * 
         * @param msg is the message received from /lidar
         */
        void scan_data_callback(
            const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        /**
         * @brief this function is called each time a message is received via 
         * the topic /robotpose. It updates robot_pose_member variable
         * 
         * @param msg is the message received from /robotpose
         */
        void robot_pose_callback(const std_msgs::msg::Float32::SharedPtr msg);

        /**
         * @brief this function is called each time a message is received via 
         * the topic /green_density. It updates density_ variable
         * 
         * @param msg is the message received from /green_density
         */
        void density_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // the current state of the system
        int current_state_;

        // wall finding member variables
        double start_pose_;
        double min_distance_pose_;
        double min_distance_;
        
        // choice of wall to follow (left or right wall)
        WallFollowChoice wall_choice_;

        // track poses that need to be remembered
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

        // a ros clock
        rclcpp::Clock ros_clk;
        rclcpp::Time current_time;

        //TODO
        bool locate_flag_;
    
        /**
         * @brief determines the robot's current position relative
         * it's surroundings walls and chooses an appropriate next state based
         * on this
         */
        void GET_TB3_DIRECTION_logic();

        /**
         * @brief sequentially searches for and traverses to the enarest wall
         */
        void LOCATE_WALL_logic();

        /**
         * @brief controls the robot so it can spin in the spot and begin to 
         * search for the nearest wall
         */
        void ROTATE_IN_PLACE_logic();
};