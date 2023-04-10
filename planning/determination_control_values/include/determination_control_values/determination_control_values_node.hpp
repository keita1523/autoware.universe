#include <stdio.h> //need?
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/odometry.hpp"

// Autoware
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

// Whill
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace motion_planning
{

  using autoware_auto_perception_msgs::msg::PredictedObjects;
  using sensor_msgs::msg::Joy;
  using Odometry = nav_msgs::msg::Odometry;

  // for feedback from whill to Autoware
  using AwSteering = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using AwVelocity = autoware_auto_vehicle_msgs::msg::VelocityReport;

  using geometry_msgs::msg::PolygonStamped;

class DeterminationControlValues : public rclcpp::Node
{
public:
  explicit DeterminationControlValues(const rclcpp::NodeOptions & node_options);

private:
  void detectionResultCallback(const autoware_auto_perception_msgs::msg::PredictedObjects  msg);
  void detectionPointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void joyStatusCallback(const sensor_msgs::msg::Joy msg);
  void controlWhillVehicle(sensor_msgs::msg::Joy joy_);
  void callbackWhillOdom(const Odometry::ConstSharedPtr & msg);

  bool transCoordinate(
		const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer);
    //,geometry_msgs::msg::Point & self_pose);

  void publishDebugArea(tier4_autoware_utils::LinearRing2d footprint, const double z);

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr sub_detection_results_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_detection_points_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_status_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_control_status_;

  // for feedback from whill to Autoware
  rclcpp::Subscription<Odometry>::SharedPtr sub_whill_odom_;
  rclcpp::Publisher<AwVelocity>::SharedPtr pub_velocity_status_;
  rclcpp::Publisher<AwSteering>::SharedPtr pub_steering_status_;

  rclcpp::Publisher<PolygonStamped>::SharedPtr pub_debug_detection_area_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
	tf2_ros::TransformListener tf_listener_{tf_buffer_};

  struct Object{
    geometry_msgs::msg::Pose obstacle_pose_;
    geometry_msgs::msg::Pose lidar_pose_;
    double distance_;
  };
  Object object_;

  struct BoundaryCondition{
    bool  detection_objects_;
    bool  detection_points_; 
    bool  is_publishing_debug_detection_area_;
    double detection_margin_max_;
    double detection_margin_min_;
    double detection_area_front_;
    double detection_area_rear_;
    double detection_area_left_;
    double detection_area_right_;
    long detection_point_num_min_;  
  };
  BoundaryCondition boundary_condition_;

  bool emagency_stop = false;

};

} // namespace