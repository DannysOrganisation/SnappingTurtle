/*
Odometry node IMPLEMENTATION
*/

#include "odom.hpp"

using namespace std::chrono_literals;

Odom::Odom() : Node("Odometry_Node")
{   

    // initialise the current pose
    robot_pose_ = 0.0;
    prev_robot_pose_ = 0.0;

    // Initialise subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", /* subscribe to topic /scan */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &Odom::odom_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

    //intialise publisher
    odom_pub_ = this->create_publisher<std_msgs::msg::Float32>("robotpose", STANDARD_BUFFER_SIZE);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("turtlebot_path", STANDARD_BUFFER_SIZE);


    // create the timer that will cotrol how often the state gets published
    update_timer_ = this->create_wall_timer(20ms, std::bind(&Odom::update_pose, this));

    // display successful creation message
    RCLCPP_INFO(this->get_logger(), "Odometry_Node has been successfully initialised");
}

Odom::~Odom()
{
  RCLCPP_INFO(this->get_logger(), "Odometry_Node has been terminated");
}


void Odom::update_pose()
{
  auto msg = std_msgs::msg::Float32();
  msg.data = robot_pose_;
  odom_pub_->publish(msg);




}


/*
The following function takes in the robots odometry and outputs stores the
yaw direction of the robot in 'robot_pose' 

@param msg: a nav_msgs::msg::Odometry::SharedPtr that corresponds to a quaternion (4 dimensional)

@return yaw: The current yaw angle in radians of the robot.
*/
void Odom::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  // save the previous result
  prev_robot_pose_ = robot_pose_;

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

  //  logging statement to establish that this works
  // RCLCPP_INFO(this->get_logger(), "Pose: '%f'", yaw);

  // set the member variable to the yaw (robot should only have rotation in this dimension)
  robot_pose_ = yaw;


  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.pose = msg->pose.pose;

  path_.header = msg->header;
  path_.poses.push_back(pose_stamped);
  path_pub_->publish(path_);
}


/*
Getter for the robot pose. 
Const used such that the getter can't change the robot
pose itself.

@return: robot_pose_ (the current yaw)
*/
double Odom::get_robot_pose() const
{
    return robot_pose_;
}

#ifdef ODOM_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom>());
   rclcpp::shutdown();
}
#endif