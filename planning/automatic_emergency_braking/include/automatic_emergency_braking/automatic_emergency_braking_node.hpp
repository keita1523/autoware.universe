# include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "tier4_planning_msgs/msg/stop_reason.hpp"
#include "tier4_planning_msgs/msg/stop_reason_array.hpp"

#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>


#include <sensor_msgs/msg/joy.hpp>

namespace motion_planning
{

using sensor_msgs::msg::Joy;
using Odometry = nav_msgs::msg::Odometry;
using AwSteering = autoware_auto_vehicle_msgs::msg::SteeringReport;
using AwVelocity = autoware_auto_vehicle_msgs::msg::VelocityReport;
class AutomaticEmergencyBrakingNode : public rclcpp::Node
{
public:
  explicit AutomaticEmergencyBrakingNode(const rclcpp::NodeOptions & node_options);
  struct Param
  {
    double min;
    double max;
  };

private:
  void currentTwistCallback(const Odometry::ConstSharedPtr input_msg);

	void subStopReasons(const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr msg);
  void joyStatusCallback(const sensor_msgs::msg::Joy msg);
  void controlWhillVehicle(sensor_msgs::msg::Joy joy_);
  void callbackWhillOdom(const Odometry::ConstSharedPtr & msg);

	rclcpp::Subscription<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr sub_stop_reasons_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_current_twist_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_status_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_control_status_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_whill_odom_;
  rclcpp::Publisher<AwVelocity>::SharedPtr pub_velocity_status_;
  rclcpp::Publisher<AwSteering>::SharedPtr pub_steering_status_;
  rclcpp::Publisher<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr pub_stop_reason;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_twist_;
  Param param_;
  bool emagency_stop = false;

};

} // namespace sample_module