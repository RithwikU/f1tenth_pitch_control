#include "rclcpp/rclcpp.hpp"
#include "simulator/pitch_controller.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;

class PitchController : public rclcpp::Node {
    // Implement pitch controller
private:

    std::string drive_topic_ = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

public:
    PitchController():
        Node("pitch_controller_node")
    {
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, 1);
        // Experiment in simulation
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = 1.0;
        drive_msg.drive.steering_angle = 0.0;
        drive_pub_->publish(drive_msg);
        RCLCPP_INFO(this->get_logger(), "Published a message: '%d'", drive_msg.drive.speed);
    }

    ~PitchController() {}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PitchController>());
    rclcpp::shutdown();
    cout << "Pitch controller node has shut down.LOL" << endl;
    return 0;
}