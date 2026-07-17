// Trajectory Tracking 轨迹跟踪节点:
// 加载/校验轨迹(1~4)、状态机、最近点/误差/进度计算;
// 订阅 /nav/pose,发布 /nav/traj/reference(TrackingReference);
// 服务 /nav/traj/cmd(仅供 Mission Manager 调用)。
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "tower_nav/trajectory_manager.hpp"
#include "tower_nav_msgs/msg/tracking_reference.hpp"
#include "tower_nav_msgs/srv/trajectory_command.hpp"

namespace tower_nav
{

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode()
  : Node("trajectory_node")
  {
    TrajectoryManager::Limits lim;
    lim.max_point_gap = declare_parameter("max_point_gap", 1.0);
    lim.goal_tolerance = declare_parameter("goal_tolerance", 0.15);
    manager_ = std::make_unique<TrajectoryManager>(lim);

    traj_dir_ = declare_parameter("trajectory_dir", std::string(""));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/nav/pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        onPose(msg);
      });

    ref_pub_ = create_publisher<tower_nav_msgs::msg::TrackingReference>(
      "/nav/traj/reference", rclcpp::QoS(10));

    cmd_srv_ = create_service<tower_nav_msgs::srv::TrajectoryCommand>(
      "/nav/traj/cmd",
      [this](const tower_nav_msgs::srv::TrajectoryCommand::Request::SharedPtr req,
      tower_nav_msgs::srv::TrajectoryCommand::Response::SharedPtr res) {
        onCommand(req, res);
      });

    RCLCPP_INFO(get_logger(), "轨迹节点已启动:轨迹目录 %s", traj_dir_.c_str());
  }

private:
  void onPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);

    const bool was_running = manager_->state() == TrajState::kRunning;
    const TrackingRef ref = manager_->update(
      msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

    tower_nav_msgs::msg::TrackingReference out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "odom";
    out.point.x = ref.point.x;
    out.point.y = ref.point.y;
    out.point.yaw = ref.point.yaw;
    out.point.v_ref = ref.point.v_ref;
    out.point.curvature = ref.point.curvature;
    out.lateral_error = ref.lateral_error;
    out.heading_error = ref.heading_error;
    out.progress = manager_->progress();
    out.traj_id = manager_->trajId();
    out.tracking_active = manager_->state() == TrajState::kRunning;
    ref_pub_->publish(out);

    if (was_running && manager_->state() == TrajState::kFinished) {
      RCLCPP_INFO(get_logger(), "轨迹 %u 到达终点,状态 FINISHED", manager_->trajId());
    }
  }

  void onCommand(
    const tower_nav_msgs::srv::TrajectoryCommand::Request::SharedPtr req,
    tower_nav_msgs::srv::TrajectoryCommand::Response::SharedPtr res)
  {
    using Srv = tower_nav_msgs::srv::TrajectoryCommand;
    bool ok = false;
    std::string msg;
    switch (req->command) {
      case Srv::Request::LOAD: {
          const std::string path = traj_dir_ + "/traj_" +
            std::to_string(req->traj_id) + ".yaml";
          ok = manager_->loadFromFile(req->traj_id, path);
          msg = ok ? "轨迹 " + std::to_string(req->traj_id) + " 加载成功" :
            manager_->lastError();
          break;
        }
      case Srv::Request::START:
        ok = manager_->start();
        msg = ok ? "轨迹开始跟踪" : manager_->lastError();
        break;
      case Srv::Request::PAUSE:
        ok = manager_->pause();
        msg = ok ? "轨迹暂停" : manager_->lastError();
        break;
      case Srv::Request::RESUME:
        ok = manager_->resume();
        msg = ok ? "轨迹恢复" : manager_->lastError();
        break;
      case Srv::Request::STOP:
        ok = manager_->stop();
        msg = ok ? "轨迹停止" : manager_->lastError();
        break;
      case Srv::Request::RESET:
        // FAULT→IDLE;非 FAULT 态复位视为幂等成功(便于任务复位流程)
        if (manager_->state() == TrajState::kFault) {
          ok = manager_->reset();
          msg = ok ? "轨迹复位" : manager_->lastError();
        } else {
          ok = true;
          msg = "轨迹无故障,无需复位";
        }
        break;
      default:
        msg = "未知命令";
        break;
    }
    if (!ok) {
      RCLCPP_WARN(get_logger(), "轨迹命令失败: %s", msg.c_str());
    }
    res->success = ok;
    res->message = msg;
    res->traj_state = static_cast<uint8_t>(manager_->state());
  }

  std::unique_ptr<TrajectoryManager> manager_;
  std::string traj_dir_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<tower_nav_msgs::msg::TrackingReference>::SharedPtr ref_pub_;
  rclcpp::Service<tower_nav_msgs::srv::TrajectoryCommand>::SharedPtr cmd_srv_;
};

}  // namespace tower_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tower_nav::TrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
