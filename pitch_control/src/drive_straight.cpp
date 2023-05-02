#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class StraightPublisher : public rclcpp::Node
{
  public:
    StraightPublisher()
    : Node("straight_pub_node"), count_(0)
    {
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("teleop", 10);
      timer_ = this->create_wall_timer(
      5ms, std::bind(&StraightPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "base_link";
        message.drive.speed = double(count_/50) + 0.5;
        message.drive.speed = message.drive.speed > 8.0 ? 8.0 : message.drive.speed;
        message.drive.steering_angle = 0.0;
        RCLCPP_INFO(this->get_logger(), "Speed: '%f'", message.drive.speed);
        publisher_->publish(message);
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StraightPublisher>());
  rclcpp::shutdown();
  return 0;
}