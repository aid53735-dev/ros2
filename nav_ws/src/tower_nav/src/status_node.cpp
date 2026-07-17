// Status Manager 状态管理节点:
// 汇聚各节点信息发布 /nav/status:机器人/任务/轨迹状态、进度、Pose、速度、跟踪误差、故障码。
#include <chrono>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "tower_nav_msgs/msg/fault.hpp"
#include "tower_nav_msgs/msg/nav_status.hpp"
#include "tower_nav_msgs/msg/tracking_reference.hpp"

namespace tower_nav
{

using namespace std::chrono_literals;

class StatusNode : public rclcpp::Node
{
public:
  StatusNode()
  : Node("status_node")
  {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/nav/pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        status_.pose = *msg;
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        status_.velocity = *msg;
      });
    ref_sub_ = create_subscription<tower_nav_msgs::msg::TrackingReference>(
      "/nav/traj/reference", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::TrackingReference::ConstSharedPtr msg) {
        status_.traj_id = msg->traj_id;
        status_.progress = msg->progress;
        status_.lateral_error = msg->lateral_error;
        status_.heading_error = msg->heading_error;
        // 轨迹状态由 tracking_active/progress 推断之外,精确值来自服务响应;
        // 此处采用 RUNNING/非 RUNNING 简化映射,详细任务态见 mission_state
        status_.traj_state = msg->tracking_active ?
          tower_nav_msgs::msg::NavStatus::TRAJ_RUNNING :
          status_.traj_state;
      });
    mission_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/nav/mission_state", rclcpp::QoS(10),
      [this](std_msgs::msg::UInt8::ConstSharedPtr msg) {
        status_.mission_state = msg->data;
        // 任务态映射轨迹显示态(HOLDING→PAUSED 等)
        using NS = tower_nav_msgs::msg::NavStatus;
        switch (msg->data) {
          case NS::MISSION_EXECUTING: status_.traj_state = NS::TRAJ_RUNNING; break;
          case NS::MISSION_HOLDING: status_.traj_state = NS::TRAJ_PAUSED; break;
          case NS::MISSION_COMPLETED: status_.traj_state = NS::TRAJ_FINISHED; break;
          case NS::MISSION_ERROR: status_.traj_state = NS::TRAJ_FAULT; break;
          default: break;
        }
      });
    loc_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav/loc_valid", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        status_.localization_valid = msg->data;
      });
    fault_sub_ = create_subscription<tower_nav_msgs::msg::Fault>(
      "/nav/fault", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::Fault::ConstSharedPtr msg) {
        status_.fault_code = msg->code;
      });

    status_pub_ = create_publisher<tower_nav_msgs::msg::NavStatus>(
      "/nav/status", rclcpp::QoS(10));

    // 10 Hz 汇聚发布
    timer_ = create_wall_timer(
      100ms, [this]() {
        status_.header.stamp = get_clock()->now();
        status_pub_->publish(status_);
      });

    RCLCPP_INFO(get_logger(), "状态管理节点已启动(/nav/status @10Hz)");
  }

private:
  tower_nav_msgs::msg::NavStatus status_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<tower_nav_msgs::msg::TrackingReference>::SharedPtr ref_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loc_valid_sub_;
  rclcpp::Subscription<tower_nav_msgs::msg::Fault>::SharedPtr fault_sub_;
  rclcpp::Publisher<tower_nav_msgs::msg::NavStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tower_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tower_nav::StatusNode>());
  rclcpp::shutdown();
  return 0;
}
