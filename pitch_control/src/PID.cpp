#include "pitch_control/PID.hpp"

PID::PID() : Node("pid_node"),
                m_roll(0.0), m_pitch(0.0), m_yaw(0.0),
                m_prev_error(0.0), m_I_sum(0.0)
{

    // Subscribe to the Odometry messages
    mp_imu_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    self->imu_topic, 1, std::bind(&PID::odom_callback, this, std::placeholders::_1));

    // Create a publisher to drive
    mp_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(self->drive_topic, 1);

    // Make a parameter (example below)
    // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    // param_desc.description = "Threshold value that is used to determine if the iTTC required automatic emergency braking";
    // this->declare_parameter("m_thresh", 1.15f, param_desc);  // Speed
    // this->get_parameter("m_thresh").get_parameter_value().get<double>()

    this->declare_parameter("kp", 4.47);
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Proportional gain constant in the PID controller";

    this->declare_parameter("kd", 0.099);
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Derivative gain constant in the PID controller";

    this->declare_parameter("ki", 0.0);
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Integral gain constant in the PID controller (likely not used in favor of PD controller)";

    this->declare_parameter("phi_des", 0.0);
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Desired pitch landing angle of the vehicle";

}

// void PID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
void PID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // get the current pose
    // this->m_roll = msg->pose.pose.angular.x; // roll from the imu
    // this->m_pitch = msg->pose.pose.angular.x; // pitch from the imu
    // this->m_yaw = msg->pose.pose.angular.x; //  yaw from the imu

    // get the error between the desired pose and the current state estimate
    // this->m_q0 = msg.pose.pose.quaternion
    this->m_quat(msg.pose.pose.quaternion.w,
                 msg.pose.pose.quaternion.x,
                 msg.pose.pose.quaternion.y,
                 msg.pose.pose.quaternion.z);

    // x is the 0 idx and 2 is the z idx 
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); // maybe put auto? or 3f?

    double error = this->get_parameter("phi_des").get_parameter_value().get<double>() - euler[0];
    double dt = msg.header.stamp - m_prev_time;
    this->m_I_sum += this->get_parameter("ki").get_parameter_value().get<double>() * error * dt;

    double control = this->get_parameter("kp").get_parameter_value().get<double>() * error +
                     this->get_parameter("kd").get_parameter_value().get<double>() * (error - m_prev_error) / dt +
                     this->m_I_sum;

    this->m_prev_error = error; 
    this->m_prev_time = msg.header.stamp;

    // publish if the ttc is less than the threshold and if hit_count is enough
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = control;
    m_publisher->publish(drive_msg);
    // RCLCPP_ERROR(this->get_logger(), "BOOM!"); // INFO, ERROR, WARN
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PID>());
    rclcpp::shutdown();
    return 0;
}