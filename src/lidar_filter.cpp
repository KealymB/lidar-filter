#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter() : Node("lidar_filter")
  {
    this->declare_parameter<int>("intensity_threshold", 1000);
    this->declare_parameter<int>("message_throttle", 0);
    intensity_threshold_ = this->get_parameter("intensity_threshold").as_int();
    message_throttle_ = this->get_parameter("message_throttle").as_int();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_topic",
        10,
        std::bind(&LidarFilter::lidarDataReceivedCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_lidar_topic", 10);

    RCLCPP_INFO(this->get_logger(), "Filtering out messages with an intensity below %i.", intensity_threshold_);
    RCLCPP_INFO(this->get_logger(), "Skipping every %ith message", message_throttle_);
  }

private:
  void lidarDataReceivedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    message_count_++;

    if (isThrottleDisabled() || shouldThrottleMessage())
    {
      filterLidarData(msg);
    }
  }

  bool shouldThrottleMessage()
  {
    return message_count_ % message_throttle_ != 0;
  }

  bool isThrottleDisabled()
  {
    return message_throttle_ == message_throttle_disabled_;
  }

  void filterLidarData(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 filtered_point_cloud = *msg;

    sensor_msgs::PointCloud2Modifier modifier(filtered_point_cloud);
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(filtered_point_cloud, "intensity");

    size_t index = 0;
    size_t remaining_points = 0;

    for (size_t i = 0; i < filtered_point_cloud.width; ++i)
    {
      if (*iter_intensity >= intensity_threshold_)
      {
        remaining_points++;
        index += filtered_point_cloud.point_step;
      }
      else
      {
        removePoint(filtered_point_cloud, index);
      }
      iter_intensity += 1;
    }

    filtered_point_cloud.width = remaining_points;

    publisher_->publish(filtered_point_cloud);
  }

  void removePoint(sensor_msgs::msg::PointCloud2 &point_cloud, size_t index)
  {
    for (size_t i = 0; i < point_cloud.point_step; ++i)
    {
      point_cloud.data.erase(point_cloud.data.begin() + index);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  int intensity_threshold_ = 1000;
  int message_throttle_;
  int message_throttle_disabled_ = 0;
  int message_count_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();

  return 0;
}
