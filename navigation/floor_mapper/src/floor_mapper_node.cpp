#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class FloorMapperNode : public rclcpp::Node
{
public:
  FloorMapperNode()
  : Node("floor_mapper")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/obstacle_points");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/floor_map");
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    rolling_window_ = declare_parameter<bool>("rolling_window", true);
    resolution_ = declare_parameter<double>("resolution", 0.10);
    map_width_m_ = declare_parameter<double>("map_width_m", 20.0);
    map_height_m_ = declare_parameter<double>("map_height_m", 20.0);
    origin_x_ = declare_parameter<double>("origin_x", -10.0);
    origin_y_ = declare_parameter<double>("origin_y", -10.0);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", -0.20);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 0.60);
    min_obstacle_range_ = declare_parameter<double>("min_obstacle_range", 0.20);
    max_obstacle_range_ = declare_parameter<double>("max_obstacle_range", 8.0);
    body_exclusion_radius_ = declare_parameter<double>("body_exclusion_radius", 0.45);
    occupancy_value_ = declare_parameter<int>("occupancy_value", 100);
    
    // 【新增】历史队列累积时间，默认保存最近 1.5 秒的点云，填满地图
    history_duration_ = declare_parameter<double>("history_duration", 1.5);
    free_space_value_ = declare_parameter<int>("free_space_value", 0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, qos);
    
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        handle_pointcloud(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "floor_mapper started. history_duration=%.2f s",
      history_duration_);
  }

private:
  void handle_pointcloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (resolution_ <= 0.0 || map_width_m_ <= 0.0 || map_height_m_ <= 0.0) {
      return;
    }

    const auto width_cells = static_cast<std::uint32_t>(std::ceil(map_width_m_ / resolution_));
    const auto height_cells = static_cast<std::uint32_t>(std::ceil(map_height_m_ / resolution_));
    if (width_cells == 0U || height_cells == 0U) {
      return;
    }

    // 【新增】更新点云历史队列
    rclcpp::Time current_time(msg.header.stamp);
    cloud_history_.push_back(msg);

    // 【新增】剔除过期的点云帧
    while (!cloud_history_.empty()) {
      rclcpp::Time old_time(cloud_history_.front().header.stamp);
      if ((current_time - old_time).seconds() > history_duration_) {
        cloud_history_.pop_front();
      } else {
        break;
      }
    }

    const auto map_origin = compute_map_origin();
    const auto robot_position = compute_robot_position();
    const double max_range_sq = max_obstacle_range_ * max_obstacle_range_;
    const double min_range_sq = min_obstacle_range_ * min_obstacle_range_;
    const double body_exclusion_sq = body_exclusion_radius_ * body_exclusion_radius_;
    
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = msg.header.stamp;
    grid.header.frame_id = output_frame_;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = width_cells;
    grid.info.height = height_cells;
    grid.info.origin.position.x = map_origin.first;
    grid.info.origin.position.y = map_origin.second;
    grid.info.origin.orientation.w = 1.0;
    // 重置空白，将随着历史队列中的所有点云填充成实体障碍
    grid.data.assign(static_cast<std::size_t>(width_cells) * static_cast<std::size_t>(height_cells), free_space_value_);
    
    // 【核心】将 N 秒历史内所有有效的扫描点全部投影一遍，避免局部断点和闪烁
    for (const auto & cloud_msg : cloud_history_) {
      sensor_msgs::PointCloud2ConstIterator<float> x_iter(cloud_msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iter(cloud_msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iter(cloud_msg, "z");
      for (; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
        const double x = *x_iter;
        const double y = *y_iter;
        const double z = *z_iter;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z < min_obstacle_height_ || z > max_obstacle_height_) {
          continue;
        }

        const double dx = x - robot_position.first;
        const double dy = y - robot_position.second;
        const double range_sq = dx * dx + dy * dy;
        if (range_sq < min_range_sq || range_sq > max_range_sq) {
          continue;
        }
        if (range_sq < body_exclusion_sq) {
          continue;
        }

        const int cell_x = static_cast<int>(std::floor((x - map_origin.first) / resolution_));
        const int cell_y = static_cast<int>(std::floor((y - map_origin.second) / resolution_));
        if (cell_x < 0 || cell_y < 0) {
          continue;
        }
        if (cell_x >= static_cast<int>(width_cells) || cell_y >= static_cast<int>(height_cells)) {
          continue;
        }

        const std::size_t index =
          static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(width_cells) +
          static_cast<std::size_t>(cell_x);
        grid.data[index] = static_cast<int8_t>(std::clamp(occupancy_value_, 0, 100));
      }
    }

    map_publisher_->publish(grid);
  }

  std::pair<double, double> compute_map_origin() const
  {
    if (rolling_window_ && has_odom_) {
      return {
        latest_odom_.pose.pose.position.x - (map_width_m_ * 0.5),
        latest_odom_.pose.pose.position.y - (map_height_m_ * 0.5)};
    }
    return {origin_x_, origin_y_};
  }

  std::pair<double, double> compute_robot_position() const
  {
    if (has_odom_) {
      return {latest_odom_.pose.pose.position.x, latest_odom_.pose.pose.position.y};
    }
    return {0.0, 0.0};
  }

  std::string input_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string output_frame_;
  bool rolling_window_{true};
  double resolution_{0.10};
  double map_width_m_{20.0};
  double map_height_m_{20.0};
  double origin_x_{-10.0};
  double origin_y_{-10.0};
  double min_obstacle_height_{-0.20};
  double max_obstacle_height_{0.60};
  double min_obstacle_range_{0.20};
  double max_obstacle_range_{8.0};
  double body_exclusion_radius_{0.45};
  int occupancy_value_{100};
  int free_space_value_{0};
  double history_duration_{1.5};
  bool has_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  
  // 新增的历史点云队列对象
  std::deque<sensor_msgs::msg::PointCloud2> cloud_history_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloorMapperNode>());
  rclcpp::shutdown();
  return 0;
}
