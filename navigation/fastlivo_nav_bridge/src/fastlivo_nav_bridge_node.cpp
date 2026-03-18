#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

class FastlivoNavBridgeNode : public rclcpp::Node
{
public:
  FastlivoNavBridgeNode()
  : Node("fastlivo_nav_bridge")
  {
    fastlivo_odom_topic_ = declare_parameter<std::string>(
      "fastlivo_odom_topic", "/aft_mapped_to_init");
    fastlivo_cloud_topic_ = declare_parameter<std::string>(
      "fastlivo_cloud_topic", "/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/obstacle_points");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base");
    publish_identity_map_to_odom_ = declare_parameter<bool>("publish_identity_map_to_odom", true);

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    obstacle_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    fastlivo_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      fastlivo_odom_topic_, 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        handle_fastlivo_odom(*msg);
      });

    fastlivo_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      fastlivo_cloud_topic_, 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        handle_fastlivo_cloud(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "fastlivo_nav_bridge started. input_odom=%s input_cloud=%s odom_topic=%s obstacle_topic=%s",
      fastlivo_odom_topic_.c_str(),
      fastlivo_cloud_topic_.c_str(),
      odom_topic_.c_str(),
      obstacle_topic_.c_str());
  }

private:
  void handle_fastlivo_odom(const nav_msgs::msg::Odometry & msg)
  {
    if (publish_identity_map_to_odom_) {
      publish_identity_map_to_odom(msg.header.stamp);
    }

    nav_msgs::msg::Odometry nav_odom = msg;
    nav_odom.header.frame_id = odom_frame_;
    nav_odom.child_frame_id = base_frame_;
    odom_publisher_->publish(nav_odom);

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = msg.header.stamp;
    odom_to_base.header.frame_id = odom_frame_;
    odom_to_base.child_frame_id = base_frame_;
    odom_to_base.transform.translation.x = msg.pose.pose.position.x;
    odom_to_base.transform.translation.y = msg.pose.pose.position.y;
    odom_to_base.transform.translation.z = msg.pose.pose.position.z;
    odom_to_base.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_to_base);
  }

  void handle_fastlivo_cloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    sensor_msgs::msg::PointCloud2 obstacle_cloud = msg;
    obstacle_cloud.header.frame_id = map_frame_;
    obstacle_publisher_->publish(obstacle_cloud);
  }

  void publish_identity_map_to_odom(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_to_odom);
  }

  std::string fastlivo_odom_topic_;
  std::string fastlivo_cloud_topic_;
  std::string odom_topic_;
  std::string obstacle_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_identity_map_to_odom_{true};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlivo_odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr fastlivo_cloud_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlivoNavBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
