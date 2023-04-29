#include <chrono>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/vesc.hpp"

// I think the following is for the imu message? I cant tell without the sim setup
#include "vesc_driver/vesc_driver.h"
#include <vesc_msgs/VescStateStamped.h>


// #include "geometry_msgs/msg/"
// #include "std_msgs/msg/string.h"

using namespace Eigen; // for the eigen functions

class PID : public rclcpp::Node {
// The class that handles PID for pitch control in mid-air

public:
    // Default Constructor
    PID();

private:
    // double m_roll; // m_ is being used to denote member variables
    // double m_pitch;
    // double m_yaw;
    double m_prev_error; 
    float m_prev_time;
    double m_I_sum;
    
    // quaternion messages from the pose
    // geometry_msgs::msg::Quaternion m_q0; // quaternion from the odometry message
    Eigen::Quaterniond m_quat; // quaternion from the Eigen package

    // std::string imu_topic = "/ego_racecar/odom";
    std::string imu_topic = "/sensors/imu"; // /raw";
    std::string drive_topic = "/drive";
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mp_imu_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr mp_drive_pub_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
};