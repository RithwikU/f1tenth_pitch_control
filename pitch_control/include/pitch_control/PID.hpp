#include <chrono>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
// #include "nav_msgs/msg/vesc.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include <typeinfo>

// I think the following is for the imu message? I cant tell without the sim setup
// #include "vesc_driver/vesc_driver.h"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
// #include "vesc_msgs"


// #include "geometry_msgs/msg/"
// #include "std_msgs/msg/string.h"

// using namespace Eigen3; // for the eigen functions

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
    double m_prev_time;
    double m_I_sum;
    
    // quaternion messages from the pose
    // geometry_msgs::msg::Quaternion m_q0; // quaternion from the odometry message
    // Eigen3::Quaterniond m_quat; // quaternion from the Eigen package

    // std::string imu_topic = "/ego_racecar/odom";
    std::string imu_topic = "/sensors/imu"; // /raw";
    // std::string imu_topic = "/sensors/imu/raw";
    std::string drive_topic = "/drive";
    std::string error_topic = "/pitch_error";
    
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mp_imu_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mp_imu_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr mp_imu_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr mp_drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mp_error_pub_;

    // void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    // void odom_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void odom_callback(const vesc_msgs::msg::VescImuStamped::ConstSharedPtr msg);
    double deg_to_rad(double angle);
};