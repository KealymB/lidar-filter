#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class LidarFilter : public rclcpp::Node
{
  int default_intensity_threshold = 1000;
public:
  LidarFilter() : Node("pointcloud_subscriber")
  {
    this->declare_parameter<int>("intensity_threshold", default_intensity_threshold);
    intensity_threshold_ = this->get_parameter("intensity_threshold").as_int();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_topic",
        10,
        std::bind(&LidarFilter::callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_lidar_topic", 10);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    publisher_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  int intensity_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();

  return 0;
}
