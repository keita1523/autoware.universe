#include "determination_control_values/determination_control_values_node.hpp"
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

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
  boundary_condition_.detection_objects_= declare_parameter("detection_objects",false); 
  boundary_condition_.detection_points_= declare_parameter("detection_points",false);
  boundary_condition_.is_publishing_debug_detection_area_ = declare_parameter("is_publishing_debug_detection_area",false);
  boundary_condition_.detection_margin_min_= declare_parameter("detection_margin_min",1.0);
  boundary_condition_.detection_margin_max_= declare_parameter("detection_margin_max",10.0);
  boundary_condition_.detection_area_front_= declare_parameter("detection_area_front",10.0);
  boundary_condition_.detection_area_rear_= declare_parameter("detection_area_rear",4.0);
  boundary_condition_.detection_area_left_= declare_parameter("detection_area_left",4.0);
  boundary_condition_.detection_area_right_= declare_parameter("detection_area_right",4.0);
  boundary_condition_.detection_point_num_min_= declare_parameter("detection_point_num_min",100);
  //set Subscriber
  if(boundary_condition_.detection_objects_)
  {
    sub_detection_results_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/objects", 1,
      std::bind(&DeterminationControlValues::detectionResultCallback, this, std::placeholders::_1),
      createSubscriptionOptions(this));
  }
  if(boundary_condition_.detection_points_)
  {
    sub_detection_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input/pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&DeterminationControlValues::detectionPointsCallback, this, std::placeholders::_1),
      createSubscriptionOptions(this));
  }
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

  if(boundary_condition_.is_publishing_debug_detection_area_)
  {
    pub_debug_detection_area_ = this->create_publisher<PolygonStamped>("~/debug/detection_area", 1);
  }
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

void DeterminationControlValues::publishDebugArea(tier4_autoware_utils::LinearRing2d footprint, const double z)
{
  if (!boundary_condition_.is_publishing_debug_detection_area_) return;

  tier4_autoware_utils::Polygon2d footprint_polygon;
  for (const auto& point : footprint) {
    boost::geometry::append(footprint_polygon.outer(), point);
  }
  boost::geometry::correct(footprint_polygon);

  PolygonStamped polygon_stamped;
  polygon_stamped.header.frame_id = "velodyne_top";
  polygon_stamped.header.stamp = Node::now();

  for (const auto& point : footprint_polygon.outer()) {
    geometry_msgs::msg::Point32 gp;
    gp.x = point.x();
    gp.y = point.y();
    gp.z = z;
    polygon_stamped.polygon.points.push_back(gp);
  }
  pub_debug_detection_area_->publish(polygon_stamped);
}

void DeterminationControlValues::detectionResultCallback(const autoware_auto_perception_msgs::msg::PredictedObjects msg)
{
  if(!boundary_condition_.detection_objects_)return;
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
  publishDebugArea(object_detection_area, object_.lidar_pose_.position.z);

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
      RCLCPP_INFO(rclcpp::get_logger("check_test"), "dist:%lf",object_.distance_); //test
    }

    if(emagency_stop)break;
  }
  return;
}

void DeterminationControlValues::detectionPointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if(!boundary_condition_.detection_points_)return;
  emagency_stop = false; //initialize
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "velodyne_top", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      object_.lidar_pose_.position.x = transform.transform.translation.x;
      object_.lidar_pose_.position.y = transform.transform.translation.y;
      object_.lidar_pose_.position.z = transform.transform.translation.z;
      object_.lidar_pose_.orientation = transform.transform.rotation;
  } catch (tf2::TransformException & e) {
    RCLCPP_INFO(rclcpp::get_logger("my_test"), "Failed to trans coordinate use topics as is.");
    emagency_stop = true;
    return;
  }

  tier4_autoware_utils::LinearRing2d object_detection_area{};
  object_detection_area.push_back(tier4_autoware_utils::Point2d{-boundary_condition_.detection_area_rear_,  boundary_condition_.detection_area_left_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{ boundary_condition_.detection_area_front_, boundary_condition_.detection_area_left_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{ boundary_condition_.detection_area_front_,-boundary_condition_.detection_area_right_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{-boundary_condition_.detection_area_rear_, -boundary_condition_.detection_area_right_});
  object_detection_area.push_back(tier4_autoware_utils::Point2d{-boundary_condition_.detection_area_rear_,  boundary_condition_.detection_area_left_});
  object_detection_area = tier4_autoware_utils::transformVector(object_detection_area, tier4_autoware_utils::pose2transform(object_.lidar_pose_));
  publishDebugArea(object_detection_area, object_.lidar_pose_.position.z);

  Eigen::Affine3f isometry = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*msg, transformed_pointcloud);
  pcl::transformPointCloud(transformed_pointcloud, transformed_pointcloud, isometry);

  long point_num = 0;
  for (const auto & p : transformed_pointcloud) {
    tier4_autoware_utils::Point2d boost_point(p.x, p.y);

    if(boost::geometry::within(boost_point, object_detection_area))
    {
      point_num++;
    }
    if(point_num >= boundary_condition_.detection_point_num_min_)
    {
      emagency_stop = true;
      RCLCPP_INFO(rclcpp::get_logger("check_test"), "emergency stop detected. all points num:%ld", transformed_pointcloud.size());//test
      break;
    }
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