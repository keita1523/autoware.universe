#include <stdio.h> //need?
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

// Autoware
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

// Whill
#include <sensor_msgs/msg/joy.hpp>

namespace motion_planning
{

using autoware_auto_perception_msgs::msg::PredictedObjects;
using sensor_msgs::msg::Joy;

class DeterminationControlValues : public rclcpp::Node
{
public:
  explicit DeterminationControlValues(const rclcpp::NodeOptions & node_options);

private:
  void detectionResultCallback(const autoware_auto_perception_msgs::msg::PredictedObjects  msg);
  void joyStatusCallback(const sensor_msgs::msg::Joy msg);
  void controlWhillVehicle(sensor_msgs::msg::Joy joy_);

  bool transCoordinate(
		const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer);
    //,geometry_msgs::msg::Point & self_pose);

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr sub_detection_results_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_status_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_control_status_;
  tf2_ros::Buffer tf_buffer_{get_clock()};
	tf2_ros::TransformListener tf_listener_{tf_buffer_};
  
  struct Object{
    geometry_msgs::msg::Point obstacle_position_;
    geometry_msgs::msg::Point lidar_position_;
    double distance_;
  };
  Object object_;

  struct BoundaryCondition{
    double detection_margin_max_;
    double detection_margin_min_;    
  };
  BoundaryCondition boundary_condition_;

  bool emagency_stop = false;

};

} // namespace 