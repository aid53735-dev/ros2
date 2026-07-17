// Exception Manager 异常管理节点:
// 集中监测:话题心跳超时(/imu/data、/wheel/encoder、/nav/pose)、定位失效、
// 横向偏差超阈、急停 /nav/estop、任务超时;
// 统一处置:安全停车(/cmd_vel 置零)+ /nav/fault 故障码上报;
// 服务 /nav/fault/reset 复位。
#include <memory>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tower_nav/exception_monitor.hpp"
#include "tower_nav_msgs/msg/fault.hpp"
#include "tower_nav_msgs/msg/nav_status.hpp"
#include "tower_nav_msgs/msg/tracking_reference.hpp"

namespace tower_nav
{

class ExceptionNode : public rclcpp::Node
{
public:
  ExceptionNode()
  : Node("exception_node")
  {
    ExceptionParams p;
    p.imu_timeout = declare_parameter("imu_timeout", 0.5);
    p.encoder_timeout = declare_parameter("encoder_timeout", 0.5);
    p.pose_timeout = declare_parameter("pose_timeout", 0.5);
    p.lateral_dev_threshold = declare_parameter("lateral_dev_threshold", 0.30);
    p.lateral_dev_hold = declare_parameter("lateral_dev_hold", 0.3);
    p.mission_timeout = declare_parameter("mission_timeout", 1200.0);
    monitor_ = std::make_unique<ExceptionMonitor>(p);

    // 启动宽限期:等待各节点上线后才开始超时监测
    startup_grace_ = declare_parameter("startup_grace", 3.0);
    start_time_ = get_clock()->now().seconds();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::QoS(50),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) {
        monitor_->feedImu(nowSec());
      });
    encoder_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/wheel/encoder", rclcpp::QoS(50),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr) {
        monitor_->feedEncoder(nowSec());
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/nav/pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {
        monitor_->feedPose(nowSec());
      });
    loc_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav/loc_valid", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        monitor_->feedLocValid(msg->data);
      });
    ref_sub_ = create_subscription<tower_nav_msgs::msg::TrackingReference>(
      "/nav/traj/reference", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::TrackingReference::ConstSharedPtr msg) {
        monitor_->setTrackingActive(msg->tracking_active);
        monitor_->feedLateralError(nowSec(), msg->lateral_error);
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav/estop", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        monitor_->feedEstop(msg->data);
      });
    status_sub_ = create_subscription<tower_nav_msgs::msg::NavStatus>(
      "/nav/status", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::NavStatus::ConstSharedPtr msg) {
        // 任务执行状态驱动任务超时监测
        const bool executing =
          msg->mission_state == tower_nav_msgs::msg::NavStatus::MISSION_EXECUTING;
        if (executing && !mission_was_executing_) {
          monitor_->missionStarted(nowSec());
        } else if (!executing && mission_was_executing_) {
          monitor_->missionEnded();
        }
        mission_was_executing_ = executing;
      });

    fault_pub_ = create_publisher<tower_nav_msgs::msg::Fault>(
      "/nav/fault", rclcpp::QoS(10));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/nav/fault/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
        const bool ok = monitor_->reset();
        res->success = ok;
        res->message = ok ? "故障已复位" : "复位失败:急停仍处于激活状态";
        if (ok) {
          RCLCPP_INFO(get_logger(), "故障复位成功");
        } else {
          RCLCPP_WARN(get_logger(), "故障复位失败:急停未解除");
        }
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(0.05), [this]() {onTimer();});

    RCLCPP_INFO(get_logger(), "异常管理节点已启动");
  }

private:
  double nowSec() {return get_clock()->now().seconds();}

  void onTimer()
  {
    const double now = nowSec();
    // 启动宽限期内只馈入,不判超时
    if (now - start_time_ < startup_grace_) {
      publishFault(FaultCode::kNone);
      return;
    }
    const FaultCode code = monitor_->check(now);
    publishFault(code);

    if (code != FaultCode::kNone) {
      // 统一处置:安全停车
      geometry_msgs::msg::Twist zero;
      cmd_pub_->publish(zero);
      if (code != last_reported_) {
        RCLCPP_ERROR(
          get_logger(), "故障触发:code=%u %s",
          static_cast<uint16_t>(code), describe(code));
      }
    } else if (last_reported_ != FaultCode::kNone) {
      RCLCPP_INFO(get_logger(), "故障已清除,系统恢复正常");
    }
    last_reported_ = code;
  }

  void publishFault(FaultCode code)
  {
    tower_nav_msgs::msg::Fault msg;
    msg.header.stamp = get_clock()->now();
    msg.code = static_cast<uint16_t>(code);
    msg.description = describe(code);
    msg.active = code != FaultCode::kNone;
    fault_pub_->publish(msg);
  }

  std::unique_ptr<ExceptionMonitor> monitor_;
  double startup_grace_{3.0};
  double start_time_{0.0};
  bool mission_was_executing_{false};
  FaultCode last_reported_{FaultCode::kNone};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loc_valid_sub_;
  rclcpp::Subscription<tower_nav_msgs::msg::TrackingReference>::SharedPtr ref_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<tower_nav_msgs::msg::NavStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<tower_nav_msgs::msg::Fault>::SharedPtr fault_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tower_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tower_nav::ExceptionNode>());
  rclcpp::shutdown();
  return 0;
}
