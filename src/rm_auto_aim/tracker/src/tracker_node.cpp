#include "tracker/tracker_node.hpp"

#include "tracker/tracker.hpp"

// STD
#include <cmath>
#include <memory>
#include <vector>

namespace rm_auto_aim
{
LightTrackerNode::LightTrackerNode(const rclcpp::NodeOptions & options)
: Node("light_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Tracker
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
  tracking_thres_ = this->declare_parameter("tracker.tracking_thres", 2);

  tracker_ = std::make_unique<Tracker>(max_armor_distance_);

  r_pos = declare_parameter("kf.r_pos", 0.05);  // 观测噪声方差
  q_pos = declare_parameter("kf.q_pos", 0.01);  // 位置噪声方差
  q_vel = declare_parameter("kf.q_vel", 0.1);   // 速度噪声方差

  // KF
  // state: xl, yl, zl, v_xl,v_yl, v_zl
  // measurement: xl, yl, zl
  // F - Process function
  auto F = [this](const double &) {
    Eigen::MatrixXd F(6, 6);
    // clang-format off
  F << 1, 0, 0, dt_, 0,   0,
       0, 1, 0, 0,   dt_, 0,
       0, 0, 1, 0,   0,   dt_,
       0, 0, 0, 1,   0,   0,
       0, 0, 0, 0,   1,   0,
       0, 0, 0, 0,   0,   1;
    // clang-format on
    return F;
  };
  // H -  observation function
  auto H = [this](const double &) {
    Eigen::MatrixXd H(3, 6);
    // clang-format off
  H << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0;
    // clang-format on
    return H;
  };
  RCLCPP_DEBUG(rclcpp::get_logger("light_tracker"), "dimension: %d", int(H(1).cols()));
  // update_Q - process noise covariance matrix
  Eigen::MatrixXd Q(6, 6);
  // clang-format off
  Q << q_pos, 0,    0,    0,    0,    0,
       0,    q_pos, 0,    0,    0,    0,
       0,    0,    q_pos, 0,    0,    0,
       0,    0,    0,    q_vel, 0,    0,
       0,    0,    0,    0,    q_vel, 0,
       0,    0,    0,    0,    0,    q_vel;
  // clang-format on

  // update_R - measurement noise covariance matrix

  Eigen::MatrixXd R(3, 3);
  // clang-format off
    R << r_pos, 0,     0,
         0,     r_pos, 0,
         0,     0,     r_pos;
  // clang-format on
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 6> p0;
  p0.setIdentity();
  tracker_->kf = KalmanFilter{F, H, Q, R, p0};

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  lights_sub_.subscribe(this, "detector/lights", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "muzzle_link");
  tf2_filter_ = std::make_shared<tf2_filter>(
    lights_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&LightTrackerNode::lightsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "tracker/target", rclcpp::SensorDataQoS());
  yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("tracker/dyaw", 10);

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.02;
  linear_v_marker_.scale.y = 0.02;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.b = 1.0;
  linear_v_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
  light_marker_.ns = "lights";
  light_marker_.type = visualization_msgs::msg::Marker::CUBE;
  light_marker_.scale.x = 0.01;
  light_marker_.scale.y = 0.045;
  light_marker_.scale.z = 0.045;
  light_marker_.color.a = 1.0;
  light_marker_.color.g = 1.0;
  light_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracker/marker", 10);

  last_time_ = this->now();
}

void LightTrackerNode::lightsCallback(const auto_aim_interfaces::msg::Lights::SharedPtr lights_msg)
{
  marker_array_.markers.clear();
  // Tranform light position from image frame to gimbal_link coordinate
  for (auto & light : lights_msg->lights) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = lights_msg->header;
    // used to bag
    // ps.header.frame_id = "camera_optical_frame";
    ps.pose = light.pose;
    try {
      light.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal lights
  lights_msg->lights.erase(
    std::remove_if(
      lights_msg->lights.begin(), lights_msg->lights.end(),
      [this](const auto_aim_interfaces::msg::Light & light) {
        return abs(light.pose.position.z) > 1.1;
      }),
    lights_msg->lights.end());

  if (lights_msg->lights.empty()) {
    return;
  } else {
    // Init message
    auto_aim_interfaces::msg::TrackerInfo info_msg;
    auto_aim_interfaces::msg::Target target_msg;
    rclcpp::Time time = lights_msg->header.stamp;
    target_msg.header.stamp = time;
    target_msg.header.frame_id = target_frame_;
    target_msg.position = lights_msg->lights[0].pose.position;
    // target_msg.id = "base";
    dt_ = (time - last_time_).seconds();
    target_msg.dt = dt_;
    target_msg_ = std::make_shared<auto_aim_interfaces::msg::Target>(target_msg);

    // RCLCPP_INFO(
    //   get_logger(), "Target: %f, %f, %f", target_msg.position.x, target_msg.position.y,
    //   target_msg.position.z);

    // Update tracker
    if (tracker_->tracker_state == Tracker::LOST) {
      tracker_->init(target_msg_);
      target_msg.tracking = false;
    } else {
      dt_ = (time - last_time_).seconds();
      tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

      tracker_->update(target_msg_);

      // Publish Info
      info_msg.position.x = tracker_->measurement(0);
      info_msg.position.y = tracker_->measurement(1);
      info_msg.position.z = tracker_->measurement(2);
      info_pub_->publish(info_msg);

      if (tracker_->tracker_state == Tracker::DETECTING) {
        target_msg.tracking = false;
      } else if (
        tracker_->tracker_state == Tracker::TRACKING ||
        tracker_->tracker_state == Tracker::TEMP_LOST) {
        target_msg.tracking = true;
        // Fill target message
        const auto & state = tracker_->target_state;
        target_msg.position.x = state(0);
        target_msg.position.y = state(1);
        target_msg.position.z = state(2);
        target_msg.velocity.x = state(3);
        target_msg.velocity.y = state(4);
        target_msg.velocity.z = state(5);

        RCLCPP_INFO(
          get_logger(), "Target: %f, %f, %f", target_msg.position.x, target_msg.position.y,
          target_msg.position.z);

        // float distance = sqrt(state(0) * state(0) + state(1) * state(1) + state(2) * state(2));
        float yaw = -atan2(state(1), state(0)) * 180.0 / M_PI;
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = yaw;
        yaw_pub_->publish(yaw_msg);

      } else if (tracker_->tracker_state == Tracker::CHANGE_TARGET) {
        target_msg.tracking = false;
      }
    }

    last_time_ = time;

    target_pub_->publish(target_msg);

    publishMarkers(target_msg);
  }
}

void LightTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  light_marker_.header = linear_v_marker_.header = target_msg.header;
  light_marker_.pose.position = target_msg.position;

  using Marker = visualization_msgs::msg::Marker;

  light_marker_.action =
    (light_marker_.header.frame_id == target_frame_) ? Marker::ADD : Marker::DELETE;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  light_marker_.pose.orientation = tf2::toMsg(q);

  linear_v_marker_.action =
    (linear_v_marker_.header.frame_id == target_frame_) ? Marker::ADD : Marker::DELETE;
  linear_v_marker_.points.clear();
  linear_v_marker_.points.emplace_back(target_msg.position);
  geometry_msgs::msg::Point arrow_end = target_msg.position;
  arrow_end.x += target_msg.velocity.x;
  arrow_end.y += target_msg.velocity.y;
  arrow_end.z += target_msg.velocity.z;
  linear_v_marker_.points.emplace_back(arrow_end);

  marker_array_.markers.emplace_back(light_marker_);
  marker_array_.markers.emplace_back(linear_v_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::LightTrackerNode)