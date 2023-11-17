#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <memory>

class LivoxPointcloud : public rclcpp::Node {
public:
  LivoxPointcloud(const rclcpp::NodeOptions& options)
  : Node("livox_pointcloud", options), tf_buffer_(this->get_clock())
  {
    transformed_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/concatenated/pointcloud", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort());
    RCLCPP_INFO(rclcpp::get_logger("livox pointcloud"), "start");

    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      auto output = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
      RCLCPP_INFO(rclcpp::get_logger("livox pointcloud"), "points num = %ld", msg->data.size());
      output->header.stamp = this->now();
      transformed_point_cloud_publisher_->publish(*output);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_point_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  tf2_ros::Buffer tf_buffer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxPointcloud>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
