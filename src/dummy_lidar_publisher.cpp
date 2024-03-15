#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class DummyLidarPublisher : public rclcpp::Node
{
public:
    DummyLidarPublisher() : Node("pointcloud_publisher")
    {
        this->declare_parameter<float>("coordinate_deviation_in_meters", 3.0);
        this->declare_parameter<int>("number_of_points", 100);
        coordinate_deviation_in_meters_ = this->get_parameter("coordinate_deviation_in_meters").as_double();
        number_of_points_ = this->get_parameter("number_of_points").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_topic", 10);

        initializePointCloud();

        publish_lidar_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DummyLidarPublisher::publishPointCloud, this));

        RCLCPP_INFO(this->get_logger(), "Creating a point cloud with %i points.", number_of_points_);
        RCLCPP_INFO(this->get_logger(), "Points are within +-%f m of the origin", coordinate_deviation_in_meters_);
    }

private:
    void initializePointCloud()
    {
        auto pointcloud_msg = sensor_msgs::msg::PointCloud2();
        pointcloud_msg_.header.frame_id = "map";
        pointcloud_msg_.height = 1;
        pointcloud_msg_.width = number_of_points_;
        pointcloud_msg_.fields = {
            createPointField("x", sensor_msgs::msg::PointField::FLOAT32, 1, 0),
            createPointField("y", sensor_msgs::msg::PointField::FLOAT32, 1, 4),
            createPointField("z", sensor_msgs::msg::PointField::FLOAT32, 1, 8),
            createPointField("intensity", sensor_msgs::msg::PointField::FLOAT32, 1, 12)};
        pointcloud_msg_.is_bigendian = false;
        pointcloud_msg_.point_step = 16;
        pointcloud_msg_.row_step = 16 * number_of_points_;
        pointcloud_msg_.is_dense = true;
        pointcloud_msg_.data.resize(pointcloud_msg_.point_step * pointcloud_msg_.width);

        sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg_);
        sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg_, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(pointcloud_msg_, "intensity");

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-coordinate_deviation_in_meters_, coordinate_deviation_in_meters_);

        for (size_t i = 0; i < pointcloud_msg_.width; ++i)
        {
            *iter_x = dis(gen);
            *iter_y = dis(gen);
            *iter_z = dis(gen);
            *iter_intensity = 500 + i * 10;

            iter_x += 1;
            iter_y += 1;
            iter_z += 1;
            iter_intensity += 1;
        }
    }

    void publishPointCloud()
    {
        pointcloud_msg_.header.stamp = this->now();

        publisher_->publish(pointcloud_msg_);
    }

    sensor_msgs::msg::PointField createPointField(const std::string &name, uint8_t datatype, uint32_t count, uint32_t offset)
    {
        sensor_msgs::msg::PointField field;

        field.name = name;
        field.datatype = datatype;
        field.count = count;
        field.offset = offset;

        return field;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_lidar_timer_;
    sensor_msgs::msg::PointCloud2 pointcloud_msg_;

    float coordinate_deviation_in_meters_;
    int number_of_points_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyLidarPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
