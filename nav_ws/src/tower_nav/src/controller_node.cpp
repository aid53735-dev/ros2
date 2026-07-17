// Navigation Controller 导航控制器节点:
// 订阅 /nav/traj/reference 与 /nav/fault;50 Hz 控制律输出 /cmd_vel;
// 前馈 ω_ff = v·κ + 反馈 ω_fb = k_y·e_y + k_θ·e_θ;安全限幅;
// FAULT / 急停 / 跟踪未激活时输出零速。
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "tower_nav/tracking_controller.hpp"
#include "tower_nav_msgs/msg/fault.hpp"
#include "tower_nav_msgs/msg/tracking_reference.hpp"

namespace tower_nav
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node")
  {
    ControlParams p;
    p.k_y = declare_parameter("k_y", 1.2);
    p.k_theta = declare_parameter("k_theta", 1.8);
    p.max_linear_vel = declare_parameter("max_linear_vel", 0.30);
    p.max_angular_vel = declare_parameter("max_angular_vel", 0.60);
    p.max_linear_acc = declare_parameter("max_linear_acc", 0.30);
    p.max_angular_acc = declare_parameter("max_angular_acc", 1.20);
    p.max_wheel_vel = declare_parameter("max_wheel_vel", 5.0);
    p.wheel_radius = declare_parameter("wheel_radius", 0.08);
    p.wheel_separation = declare_parameter("wheel_separation", 0.40);
    controller_ = std::make_unique<TrackingController>(p);

    control_rate_ = declare_parameter("control_rate", 50.0);
    ref_timeout_ = declare_parameter("reference_timeout", 0.5);

    ref_sub_ = create_subscription<tower_nav_msgs::msg::TrackingReference>(
      "/nav/traj/reference", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::TrackingReference::ConstSharedPtr msg) {
        latest_ref_ = *msg;
        last_ref_time_ = get_clock()->now().seconds();
      });

    fault_sub_ = create_subscription<tower_nav_msgs::msg::Fault>(
      "/nav/fault", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::Fault::ConstSharedPtr msg) {
        fault_active_ = msg->active && msg->code != tower_nav_msgs::msg::Fault::NONE;
        if (fault_active_) {
          controller_->forceStop();
        }
      });

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav/estop", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        estop_ = msg->data;
        if (estop_) {
          controller_->forceStop();
        }
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_), [this]() {onTimer();});

    RCLCPP_INFO(get_logger(), "控制器节点已启动:%.0f Hz", control_rate_);
  }

private:
  void onTimer()
  {
    const double now = get_clock()->now().seconds();
    const double dt = 1.0 / control_rate_;

    // 使能条件:无故障、无急停、参考新鲜且跟踪激活
    const bool ref_fresh = last_ref_time_ > 0.0 && (now - last_ref_time_) < ref_timeout_;
    const bool enabled = !fault_active_ && !estop_ && ref_fresh &&
      latest_ref_.tracking_active;

    ControlOutput out;
    if (fault_active_ || estop_) {
      // 故障/急停:强制立即零速
      controller_->forceStop();
      out = controller_->lastOutput();
    } else {
      out = controller_->update(
        latest_ref_.point.v_ref, latest_ref_.point.curvature,
        latest_ref_.lateral_error, latest_ref_.heading_error, dt, enabled);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = out.v;
    cmd.angular.z = out.omega;
    cmd_pub_->publish(cmd);
  }

  std::unique_ptr<TrackingController> controller_;
  double control_rate_{50.0};
  double ref_timeout_{0.5};
  tower_nav_msgs::msg::TrackingReference latest_ref_;
  double last_ref_time_{-1.0};
  bool fault_active_{false};
  bool estop_{false};

  rclcpp::Subscription<tower_nav_msgs::msg::TrackingReference>::SharedPtr ref_sub_;
  rclcpp::Subscription<tower_nav_msgs::msg::Fault>::SharedPtr fault_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tower_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tower_nav::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
