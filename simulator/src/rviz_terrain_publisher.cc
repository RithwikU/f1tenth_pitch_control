#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "Eigen/Dense"
#include "simulator/height_map.h"
#include "chrono"
#include "functional"
#include "memory"

using namespace std::chrono_literals;
namespace f1tenth {

class TerrainPublisher: public rclcpp::Node
{
    public:
    TerrainPublisher()
    : Node("terrain_publisher"), count_(0)
    {
    rviz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/terrain", 10);
    std::cout << "Entering callback..." << std::endl;
    msg_callback(f1tenth::HeightMap::SlopeID);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&TerrainPublisher::timer_callback, this));
    }

    void timer_callback()
    {
        msg_callback(f1tenth::HeightMap::SlopeID);
        RCLCPP_INFO(this->get_logger(), "Published map");
    }

    private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;

    void msg_callback(HeightMap::TerrainID msg_in)
    {
    // get which terrain
      auto terrain_id = static_cast<HeightMap::TerrainID>(msg_in);
      auto terrain_ = HeightMap::MakeTerrain(terrain_id);

      // x-y area patch that should be drawn in rviz
      double dxy   =  0.06;
      double x_min = -1.0;
      double x_max =  4.0;
      double y_min = -1.0;
      double y_max =  1.0;

      visualization_msgs::msg::Marker m;
      int id = 0;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.z = 0.003;
      m.ns = "terrain";
      m.header.frame_id = "map";
      m.color.r = 245./355; m.color.g  = 222./355; m.color.b  = 179./355; // wheat
      m.color.a = 0.65;

      visualization_msgs::msg::MarkerArray msg;
      double x =  x_min;
      while (x < x_max) {
        double y = y_min;
        while (y < y_max) {
          // position
          m.pose.position.x = x;
          m.pose.position.y = y;
          m.pose.position.z = terrain_->GetHeight(x,y);

          // orientation
          Eigen::Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, x, y);
          Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), n);
          m.pose.orientation.w = q.w();
          m.pose.orientation.x = q.x();
          m.pose.orientation.y = q.y();
          m.pose.orientation.z = q.z();

          // enlarge surface-path when tilting
          double gain = 1.5;
          m.scale.x = (1+gain*n.cwiseAbs().x())*dxy;
          m.scale.y = (1+gain*n.cwiseAbs().y())*dxy;


          m.id = id++;
          msg.markers.push_back(m);

          y += dxy;
        }
        x += dxy;
      }
//      RCLCPP_INFO(this->get_logger(), "I received the message: '%d'", msg.markers.size());
      rviz_pub->publish(msg);
  }

};

} // namespace f1tenth

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<f1tenth::TerrainPublisher>());
  rclcpp::shutdown();
  return 0;
}