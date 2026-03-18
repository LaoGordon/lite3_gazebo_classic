#include <memory>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class FloorMapperNode : public rclcpp::Node
{
public:
  FloorMapperNode()
  : Node("floor_mapper")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/obstacle_points");
    map_topic_ = declare_parameter<std::string>("map_topic", "/floor_map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");

    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        (void)msg;
      });

    RCLCPP_INFO(
      get_logger(),
      "floor_mapper skeleton started. input_topic=%s map_topic=%s scan_topic=%s",
      input_topic_.c_str(),
      map_topic_.c_str(),
      scan_topic_.c_str());
  }

private:
  std::string input_topic_;
  std::string map_topic_;
  std::string scan_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloorMapperNode>());
  rclcpp::shutdown();
  return 0;
}
