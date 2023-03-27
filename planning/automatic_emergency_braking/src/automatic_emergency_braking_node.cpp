#include "automatic_emergency_braking_node/automatic_emergency_braking_node_node.hpp"

namespace motion_planning // need to change
{

AutomaticEmergencyBrakingNode::AutomaticEmergencyBrakingNode(const rclcpp::NodeOptions & node_options)
: Node("automatic_emergency_braking_node", node_options)
{
  RCLCPP_INFO(this->get_logger(), "Template launched.");
  auto & p = param_;
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

  p.min = declare_parameter("min", 0.0);
  p.max = declare_parameter("max", 3.0);

	sub_stop_reasons_ = this->create_subscription<tier4_planning_msgs::msg::StopReasonArray>(
		"~/input/stop_reasons", 1,
		std::bind(&AutomaticEmergencyBrakingNode::subStopReasons, this, std::placeholders::_1));
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

  sub_current_twist_ = this->create_subscription<Odometry>(
    "~/input/odometry", 1,
    std::bind(&AutomaticEmergencyBrakingNode::currentTwistCallback, this, std::placeholders::_1));
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

  sub_joy_status_ = this->create_subscription<sensor_msgs::msg::Joy>(
		"~/input/joy", 1,
		std::bind(&AutomaticEmergencyBrakingNode::joyStatusCallback, this, std::placeholders::_1));
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
  

  sub_whill_odom_ = this->create_subscription<Odometry>(
    "~/input/whill_odom",
    rclcpp::QoS(1),
    std::bind(&AutomaticEmergencyBrakingNode::callbackWhillOdom, this, std::placeholders::_1));
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);


  // set Publisher
	pub_control_status_ = this->create_publisher<sensor_msgs::msg::Joy>("~/output/vehicle_cmd", rclcpp::QoS(1));
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

  pub_velocity_status_ = this->create_publisher<AwVelocity>("~/output/aw_velocity", 1);
  AwVelocity set_vel;
  set_vel.longitudinal_velocity = 0.0;
  set_vel.lateral_velocity = 0.0;
  set_vel.heading_rate = 0.0;
  set_vel.header.frame_id = "base_link";
  set_vel.header.stamp = Node::now();
  pub_velocity_status_->publish(set_vel);
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

  pub_steering_status_ = this->create_publisher<AwSteering>("~/output/aw_steering", 1);
  AwSteering set_steer;
  set_steer.steering_tire_angle = 0.0;
  set_steer.stamp = Node::now();
  pub_steering_status_->publish(set_steer);
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

	pub_stop_reason = this->create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reson", rclcpp::QoS(1));
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

}

void AutomaticEmergencyBrakingNode::currentTwistCallback(const Odometry::ConstSharedPtr input_msg)
{
	current_pose_ = input_msg->pose.pose;
	current_twist_ = input_msg->twist.twist;
	return;
}


void AutomaticEmergencyBrakingNode::subStopReasons(
  const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
  tier4_planning_msgs::msg::StopReasonArray output = *msg;
  pub_stop_reason->publish(output);
  std::shared_ptr<tier4_planning_msgs::msg::StopReason> reason_for_obstacle;

  // bool find_obstacle = false;
  for (auto stop_reason : msg->stop_reasons){
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d %s", __LINE__, stop_reason.reason.c_str());
    if (stop_reason.reason == "ObstacleStop" && stop_reason.stop_factors.size() != 0){
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
      reason_for_obstacle = std::make_shared<tier4_planning_msgs::msg::StopReason>(stop_reason);
			// find_obstacle = true;
    }
  }
	// if (!find_obstacle){
  if (!reason_for_obstacle){
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
		return;
  }
  if (reason_for_obstacle->stop_factors[0].stop_factor_points.size() == 0){
    emagency_stop = false;
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d size = %ld", __LINE__, reason_for_obstacle->stop_factors.size());
  auto stop_factor_points = reason_for_obstacle->stop_factors[0].stop_factor_points[0];
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE stop    x %f y %f z %f", stop_factor_points.x, stop_factor_points.y, stop_factor_points.z);
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE current x %f y %f z %f", current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
  
  auto distance = tier4_autoware_utils::calcDistance3d(stop_factor_points, current_pose_.position);
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %f", distance);
  if (distance > param_.min && distance < param_.max) {
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
    emagency_stop = true;
  }
  else {
    emagency_stop = false;
  }

}

void AutomaticEmergencyBrakingNode::callbackWhillOdom(const Odometry::ConstSharedPtr& msg)
{
  AwSteering steer_angle;
  AwVelocity velocity;
  // RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);

  double linear = msg->twist.twist.linear.x;
  double angular = msg->twist.twist.angular.z;
  // RCLCPP_INFO(this->get_logger(),"pose  x:%f, y:%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  RCLCPP_INFO(this->get_logger(),"WhillOdom velocity:%f, angle:%f", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
  if(linear <= 0.01)
  {
    velocity.longitudinal_velocity = 0;
    steer_angle.steering_tire_angle = 0;
  }else{
    // velocity.longitudinal_velocity = mps_twist;
    // // 速度・角速度から4輪のときの速度・ステア角を求める
    // steer_angle.steering_tire_angle = std::atan(rps_twist * 0.6 / mps_twist);
    velocity.longitudinal_velocity = linear;
    steer_angle.steering_tire_angle = angular;
  }
  steer_angle.stamp = Node::now();
  velocity.header.stamp = Node::now();
  velocity.header.frame_id = "base_link";
  // steer_angle.stamp = msg->header.stamp;
  // velocity.header = msg->header;
  // RCLCPP_INFO(this->get_logger(),"velocity:%f, angle:%f", velocity.longitudinal_velocity, steer_angle.steering_tire_angle);
  // RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
  pub_steering_status_->publish(steer_angle);
  pub_velocity_status_->publish(velocity);
  return;
}

void AutomaticEmergencyBrakingNode::joyStatusCallback(const sensor_msgs::msg::Joy msg)
{
  // RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
  auto joy_ = msg;
  controlWhillVehicle(joy_);
}

void AutomaticEmergencyBrakingNode::controlWhillVehicle(sensor_msgs::msg::Joy joy_)
{
  if(emagency_stop){
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "LINE %d", __LINE__);
    joy_.axes[0] = 0.0; //　Longitudinal direction
    joy_.axes[1] = 0.0; //　horizon direction
  }

  //subscribe
	pub_control_status_->publish(joy_);
  // RCLCPP_INFO(rclcpp::get_logger("my_test"), "emagency_stop is '%d'", emagency_stop);
  return;
}


} // namespace motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::AutomaticEmergencyBrakingNode)