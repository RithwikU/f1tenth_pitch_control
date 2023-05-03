#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class StraightPublisher : public rclcpp::Node
{
  public:
    StraightPublisher()
    : Node("straight_pub_node"), count_(0)
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&StraightPublisher::joy_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

  private:
    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
    {
        if(msg->buttons[6]==1)
        {
            auto message = ackermann_msgs::msg::AckermannDriveStamped();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "base_link";
            // message.drive.speed = double(count_/5)/2.0 + 2.0;

            if(count_ < 20)
              message.drive.speed = 1.0;
            else if(count_ < 35)
              message.drive.speed = 3.5;
            // else if(count_ < 45)
            //   message.drive.speed = 6.0;
            else
              message.drive.speed = 6.5;

            message.drive.speed = message.drive.speed > 8.0 ? 8.0 : message.drive.speed;
            // message.drive.speed = -message.drive.speed;
            message.drive.steering_angle = 0.0;
            RCLCPP_INFO(this->get_logger(), "Speed: '%f'", message.drive.speed);
            publisher_->publish(message);
            count_++;
        }
        else
        {
          count_ = 0;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StraightPublisher>());
  rclcpp::shutdown();
  return 0;
}
