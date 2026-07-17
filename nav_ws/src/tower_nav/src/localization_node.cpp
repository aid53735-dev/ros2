// Localization 定位融合节点:
// 订阅 /imu/data、/wheel/encoder;差速解算 v/ω;互补融合航向;里程积分位置;
// 发布 /nav/pose(PoseWithCovarianceStamped)与 TF odom→base_link;
// 数据超时或协方差发散 → /nav/loc_valid 置 false。
#include <memory>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tower_nav/localization_filter.hpp"
#include "tower_nav/localization_source.hpp"

namespace tower_nav
{

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : Node("localization_node")
  {
    const double wheel_radius = declare_parameter("wheel_radius", 0.08);
    const double wheel_separation = declare_parameter("wheel_separation", 0.40);

    FusionParams fp;
    fp.heading_alpha = declare_parameter("heading_alpha", 0.98);
    fp.sensor_timeout = declare_parameter("sensor_timeout", 0.5);
    fp.cov_diverge_threshold = declare_parameter("cov_diverge_threshold", 5.0);
    fp.pos_noise_rate = declare_parameter("pos_noise_rate", 1e-4);
    fp.yaw_noise_rate = declare_parameter("yaw_noise_rate", 1e-6);
    filter_ = std::make_unique<LocalizationFilter>(fp);

    const double x0 = declare_parameter("initial_x", 0.0);
    const double y0 = declare_parameter("initial_y", 1.0);
    const double yaw0 = declare_parameter("initial_yaw", 0.0);
    filter_->reset(x0, y0, yaw0);

    // 插件式定位源:通过注册表创建(体现插件架构的实际使用)
    registry_ = makeDefaultRegistry(wheel_radius, wheel_separation);
    odom_source_ = std::dynamic_pointer_cast<OdomSource>(registry_.create("odom"));
    imu_source_ = std::dynamic_pointer_cast<ImuSource>(registry_.create("imu"));

    encoder_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/wheel/encoder", rclcpp::QoS(50),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {onEncoder(msg);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::QoS(50),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(msg);});

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/nav/pose", rclcpp::QoS(10));
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/nav/loc_valid", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const double rate = declare_parameter("output_rate", 50.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(), "定位节点已启动:初始位姿 (%.2f, %.2f, %.2f),源: odom+imu",
      x0, y0, yaw0);
  }

private:
  double nowSec() {return get_clock()->now().seconds();}

  void onEncoder(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    if (msg->velocity.size() < 2) {
      return;
    }
    odom_source_->feedEncoder(nowSec(), msg->velocity[0], msg->velocity[1]);
    const auto m = odom_source_->latest();
    if (m) {
      filter_->feedOdomVelocity(m->stamp, m->v, m->omega);
    }
  }

  void onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const auto & q = msg->orientation;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    imu_source_->feedImu(
      nowSec(), yaw, msg->angular_velocity.z, msg->orientation_covariance[8]);
    const auto m = imu_source_->latest();
    if (m) {
      filter_->feedImuHeading(m->stamp, m->yaw, m->yaw_rate);
    }
  }

  void onTimer()
  {
    const auto stamp = get_clock()->now();
    filter_->predict(stamp.seconds());
    const auto & st = filter_->state();

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "odom";
    pose.pose.pose.position.x = st.x;
    pose.pose.pose.position.y = st.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, st.yaw);
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.orientation.w = q.w();
    pose.pose.covariance[0] = st.cov_xx;
    pose.pose.covariance[7] = st.cov_yy;
    pose.pose.covariance[35] = st.cov_yawyaw;
    pose_pub_->publish(pose);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = st.x;
    tf.transform.translation.y = st.y;
    tf.transform.rotation = pose.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    std_msgs::msg::Bool valid;
    valid.data = filter_->valid();
    valid_pub_->publish(valid);

    if (!filter_->valid() && !warned_invalid_) {
      RCLCPP_WARN(
        get_logger(), "定位失效:odom超时=%d imu超时=%d 协方差发散=%d",
        filter_->odomTimeout(), filter_->imuTimeout(), filter_->covDiverged());
      warned_invalid_ = true;
    } else if (filter_->valid()) {
      warned_invalid_ = false;
    }
  }

  std::unique_ptr<LocalizationFilter> filter_;
  SourceRegistry registry_;
  std::shared_ptr<OdomSource> odom_source_;
  std::shared_ptr<ImuSource> imu_source_;
  bool warned_invalid_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tower_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tower_nav::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
