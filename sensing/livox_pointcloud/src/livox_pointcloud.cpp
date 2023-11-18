#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <memory>
#include <chrono>
#include <ostream>

class LivoxPointcloud : public rclcpp::Node {
public:
  std::chrono::high_resolution_clock::time_point prev_time_ = std::chrono::high_resolution_clock::now();

  LivoxPointcloud(const rclcpp::NodeOptions& options)
  : Node("livox_pointcloud", options), tf_buffer_(this->get_clock())
  {
    static std::chrono::high_resolution_clock::time_point prev_time_ = std::chrono::high_resolution_clock::now();
    transformed_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/concatenated/pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    RCLCPP_INFO(rclcpp::get_logger("livox pointcloud"), "start");

    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      auto output = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
      auto now = std::chrono::high_resolution_clock::now();
      auto period = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev_time_).count();
      RCLCPP_INFO(rclcpp::get_logger("livox pointcloud"), "points num = %ld, period = %ld", msg->data.size() / msg->point_step, period);
      prev_time_ = now;
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
