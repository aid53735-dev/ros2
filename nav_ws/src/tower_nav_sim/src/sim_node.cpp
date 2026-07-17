// tower_nav_sim 仿真节点:
// 订阅 /cmd_vel,按双轮差速运动学积分位姿;
// 发布 /wheel/encoder(JointState)、/imu/data(Imu,可配噪声零偏)、
// /sim/ground_truth(Odometry 地面真值)与 TF;
// 服务 /sim/set_slip(打滑注入)、/sim/imu_enable(IMU 停发注入)。
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tower_nav_sim/diff_drive_sim.hpp"

namespace tower_nav_sim
{

using namespace std::chrono_literals;

class SimNode : public rclcpp::Node
{
public:
  SimNode()
  : Node("tower_nav_sim")
  {
    SimParams p;
    p.wheel_separation = declare_parameter("wheel_separation", 0.40);
    p.wheel_radius = declare_parameter("wheel_radius", 0.08);
    p.imu_gyro_noise_std = declare_parameter("imu_gyro_noise_std", 0.002);
    p.imu_gyro_bias = declare_parameter("imu_gyro_bias", 0.001);
    p.imu_yaw_noise_std = declare_parameter("imu_yaw_noise_std", 0.002);
    p.encoder_noise_std = declare_parameter("encoder_noise_std", 0.001);
    p.noise_seed = static_cast<uint32_t>(declare_parameter("noise_seed", 42));
    sim_ = std::make_unique<DiffDriveSim>(p);

    const double x0 = declare_parameter("initial_x", 0.0);
    const double y0 = declare_parameter("initial_y", 1.0);
    const double yaw0 = declare_parameter("initial_yaw", 0.0);
    sim_->setPose(x0, y0, yaw0);

    step_rate_ = declare_parameter("step_rate", 100.0);
    publish_rate_ = declare_parameter("sensor_rate", 100.0);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        sim_->setCommand(msg->linear.x, msg->angular.z);
      });

    encoder_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/wheel/encoder", rclcpp::QoS(10));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::QoS(10));
    gt_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/sim/ground_truth", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    slip_srv_ = create_service<std_srvs::srv::SetBool>(
      "/sim/set_slip",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        // true → 注入固定 30% 打滑;false → 关闭
        sim_->setSlipRatio(req->data ? 0.3 : 0.0);
        res->success = true;
        res->message = req->data ? "打滑注入已开启(30%)" : "打滑注入已关闭";
      });

    imu_enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "/sim/imu_enable",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        imu_enabled_ = req->data;
        res->success = true;
        res->message = req->data ? "IMU 发布已恢复" : "IMU 发布已停止(异常注入)";
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / step_rate_),
      [this]() {onTimer();});

    RCLCPP_INFO(get_logger(), "仿真节点已启动:初始位姿 (%.2f, %.2f, %.2f)", x0, y0, yaw0);
  }

private:
  void onTimer()
  {
    const double dt = 1.0 / step_rate_;
    sim_->step(dt);
    publishSensors();
  }

  void publishSensors()
  {
    const auto now = get_clock()->now();
    const auto & st = sim_->state();

    // 编码器 JointState
    const EncoderSample enc = sim_->sampleEncoder();
    sensor_msgs::msg::JointState js;
    js.header.stamp = now;
    js.name = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {enc.left_angle, enc.right_angle};
    js.velocity = {enc.left_velocity, enc.right_velocity};
    encoder_pub_->publish(js);

    // IMU(可被异常注入停发)
    if (imu_enabled_) {
      const ImuSample imu = sim_->sampleImu();
      sensor_msgs::msg::Imu m;
      m.header.stamp = now;
      m.header.frame_id = "imu_link";
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, imu.yaw);
      m.orientation.x = q.x();
      m.orientation.y = q.y();
      m.orientation.z = q.z();
      m.orientation.w = q.w();
      m.orientation_covariance[8] = 1e-4;
      m.angular_velocity.z = imu.gyro_z;
      m.angular_velocity_covariance[8] = 1e-5;
      imu_pub_->publish(m);
    }

    // 地面真值
    nav_msgs::msg::Odometry gt;
    gt.header.stamp = now;
    gt.header.frame_id = "odom";
    gt.child_frame_id = "base_link_gt";
    gt.pose.pose.position.x = st.x;
    gt.pose.pose.position.y = st.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, st.yaw);
    gt.pose.pose.orientation.x = q.x();
    gt.pose.pose.orientation.y = q.y();
    gt.pose.pose.orientation.z = q.z();
    gt.pose.pose.orientation.w = q.w();
    gt.twist.twist.linear.x = st.v;
    gt.twist.twist.angular.z = st.omega;
    gt_pub_->publish(gt);

    // TF 真值(独立帧,避免与定位 odom→base_link 冲突)
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link_gt";
    tf.transform.translation.x = st.x;
    tf.transform.translation.y = st.y;
    tf.transform.rotation = gt.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  std::unique_ptr<DiffDriveSim> sim_;
  double step_rate_{100.0};
  double publish_rate_{100.0};
  bool imu_enabled_{true};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr slip_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr imu_enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tower_nav_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tower_nav_sim::SimNode>());
  rclcpp::shutdown();
  return 0;
}
