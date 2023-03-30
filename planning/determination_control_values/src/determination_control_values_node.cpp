#include "determination_control_values/determination_control_values_node.hpp"

namespace motion_planning
{

rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}

DeterminationControlValues::DeterminationControlValues(const rclcpp::NodeOptions & node_options)
: Node("determination_control_values", node_options)
{
  RCLCPP_INFO(rclcpp::get_logger("my_test"), "detemination_launched");
  boundary_condition_.detection_margin_min_= declare_parameter("detection_margin_min",1.0);
  boundary_condition_.detection_margin_max_= declare_parameter("detection_margin_max",10.0);
  boundary_condition_.detection_area_front_= declare_parameter("detection_area_front",10.0);
  boundary_condition_.detection_area_rear_= declare_parameter("detection_area_rear",4.0);
  boundary_condition_.detection_area_left_= declare_parameter("detection_area_left",4.0);
  boundary_condition_.detection_area_right_= declare_parameter("detection_area_right",4.0);
  //set Subscriber
  sub_detection_results_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
		"~/input/objects", 1,
		std::bind(&DeterminationControlValues::detectionResultCallback, this, std::placeholders::_1),
    createSubscriptionOptions(this));

  RCLCPP_INFO(rclcpp::get_logger("my_test"), "object_check");
  sub_joy_status_ = this->create_subscription<sensor_msgs::msg::Joy>(
		"~/input/joy", 1,
		std::bind(&DeterminationControlValues::joyStatusCallback, this, std::placeholders::_1),
    createSubscriptionOptions(this));

  sub_whill_odom_ = this->create_subscription<Odometry>(
    "~/input/whill_odom",
    rclcpp::QoS(1),
    std::bind(&DeterminationControlValues::callbackWhillOdom, this, std::placeholders::_1)
  );

  // set Publisher
	pub_control_status_ = this->create_publisher<sensor_msgs::msg::Joy>("~/output/vehicle_cmd", rclcpp::QoS(1));

  pub_velocity_status_ = this->create_publisher<AwVelocity>("~/output/aw_velocity", 1);
  AwVelocity set_vel;
  set_vel.longitudinal_velocity = 0.0;
  set_vel.lateral_velocity = 0.0;
  set_vel.heading_rate = 0.0;
  set_vel.header.frame_id = "base_link";
  set_vel.header.stamp = Node::now();
  pub_velocity_status_->publish(set_vel);

  pub_steering_status_ = this->create_publisher<AwSteering>("~/output/aw_steering", 1);
  AwSteering set_steer;
  set_steer.steering_tire_angle = 0.0;
  set_steer.stamp = Node::now();
  pub_steering_status_->publish(set_steer);
}


bool DeterminationControlValues::transCoordinate(
  const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer)
{
  try {
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer.lookupTransform(
      header.frame_id, "velodyne_top", header.stamp, rclcpp::Duration::from_seconds(0.1));
    object_.lidar_pose_.position.x = transform.transform.translation.x;
    object_.lidar_pose_.position.y = transform.transform.translation.y;
    object_.lidar_pose_.position.z = transform.transform.translation.z;
    object_.lidar_pose_.orientation = transform.transform.rotation;
    return true;
  } catch (tf2::TransformException & ex) {
    return false;
  }
}

void DeterminationControlValues::detectionResultCallback(const autoware_auto_perception_msgs::msg::PredictedObjects msg)
{
  emagency_stop = false; //initialize
  auto ret = transCoordinate(msg.header, tf_buffer_);
	if (ret == false) {
    RCLCPP_INFO(rclcpp::get_logger("my_test"), "Failed to trans coordinate use topics as is.");
		emagency_stop = true;
	}
  tier4_autoware_utils::LinearRing2d object_detection_area{};
  object_detection_area.push_back(tier4_autoware_utils::Point2d{-boundary_condition_.detection_area_rear_,  boundary_condition_.detection_area_left_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{ boundary_condition_.detection_area_front_, boundary_condition_.detection_area_left_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{ boundary_condition_.detection_area_front_,-boundary_condition_.detection_area_right_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{-boundary_condition_.detection_area_rear_, -boundary_condition_.detection_area_right_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{-boundary_condition_.detection_area_rear_,  boundary_condition_.detection_area_left_});
  object_detection_area = tier4_autoware_utils::transformVector(object_detection_area, tier4_autoware_utils::pose2transform(object_.lidar_pose_));

  for(auto object: msg.objects){
    object_.obstacle_pose_ = object.kinematics.initial_pose_with_covariance.pose;
    object_.obstacle_pose_.position.x -= object_.lidar_pose_.position.x;
    object_.obstacle_pose_.position.y -= object_.lidar_pose_.position.y;
    object_.distance_ = std::sqrt(object_.obstacle_pose_.position.x*object_.obstacle_pose_.position.x+
                                  object_.obstacle_pose_.position.y*object_.obstacle_pose_.position.y);
    tier4_autoware_utils::Point2d object_point2d{object.kinematics.initial_pose_with_covariance.pose.position.x, object.kinematics.initial_pose_with_covariance.pose.position.y};

    if( (boundary_condition_.detection_margin_min_ < object_.distance_ && boundary_condition_.detection_margin_max_  > object_.distance_) &&
        (boost::geometry::within(object_point2d, object_detection_area) ) )
    {
      emagency_stop = true;
      RCLCPP_WARN(rclcpp::get_logger("check_test"), "dist:%lf,obj:(%9lf,%9lf),lid:(%9lf,%9lf),area:(%9lf,%9lf)(%9lf,%9lf)(%9lf,%9lf)(%9lf,%9lf)",object_.distance_, object_point2d.x(), object_point2d.y(), object_.lidar_pose_.position.x, object_.lidar_pose_.position.y, object_detection_area[0][0], object_detection_area[0][1], object_detection_area[1][0], object_detection_area[1][1], object_detection_area[2][0], object_detection_area[2][1], object_detection_area[3][0], object_detection_area[3][1]); //test
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("check_test"), "dist:%lf,obj:(%9lf,%9lf),lid:(%9lf,%9lf),area:(%9lf,%9lf)(%9lf,%9lf)(%9lf,%9lf)(%9lf,%9lf)",object_.distance_, object_point2d.x(), object_point2d.y(), object_.lidar_pose_.position.x, object_.lidar_pose_.position.y, object_detection_area[0][0], object_detection_area[0][1], object_detection_area[1][0], object_detection_area[1][1], object_detection_area[2][0], object_detection_area[2][1], object_detection_area[3][0], object_detection_area[3][1]); //test
    }

    if(emagency_stop)break;
  }
  return;
}

void DeterminationControlValues::joyStatusCallback(const sensor_msgs::msg::Joy msg)
{
  auto joy_ = msg;
  controlWhillVehicle(joy_);
}

void DeterminationControlValues::controlWhillVehicle(sensor_msgs::msg::Joy joy_)
{
  if(emagency_stop){
    joy_.axes[0] = 0.0; //　Longitudinal direction
    joy_.axes[1] = 0.0; //　horizon direction
    RCLCPP_INFO(rclcpp::get_logger("my_test"), "emagency_stop is '%d'", emagency_stop);
  }

  //subscribe
	pub_control_status_->publish(joy_);
  //RCLCPP_INFO(rclcpp::get_logger("my_test"), "emagency_stop is '%d'", emagency_stop);
  return;
}


void DeterminationControlValues::callbackWhillOdom(const Odometry::ConstSharedPtr& msg)
{
  AwSteering steer_angle;
  AwVelocity velocity;

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
  pub_steering_status_->publish(steer_angle);
  pub_velocity_status_->publish(velocity);
  return;
}



} // namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::DeterminationControlValues)