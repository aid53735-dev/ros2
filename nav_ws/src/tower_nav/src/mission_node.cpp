// Mission Manager 任务管理节点:
// 对外唯一控制入口:服务 /nav/mission_cmd(START/PAUSE/RESUME/STOP/RESET+轨迹号);
// 状态机 STANDBY/READY/EXECUTING/HOLDING/COMPLETED/ABORTED/ERROR;
// 编排 Trajectory(/nav/traj/cmd)与故障联动(/nav/fault、/nav/fault/reset)。
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tower_nav/mission_state_machine.hpp"
#include "tower_nav_msgs/msg/fault.hpp"
#include "tower_nav_msgs/msg/tracking_reference.hpp"
#include "tower_nav_msgs/srv/mission_command.hpp"
#include "tower_nav_msgs/srv/trajectory_command.hpp"

namespace tower_nav
{

using namespace std::chrono_literals;
using TrajSrv = tower_nav_msgs::srv::TrajectoryCommand;
using MissionSrv = tower_nav_msgs::srv::MissionCommand;

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission_node")
  {
    // 客户端放独立回调组:服务回调内同步等待客户端响应时不被互斥阻塞
    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    traj_client_ = create_client<TrajSrv>(
      "/nav/traj/cmd", rmw_qos_profile_services_default, client_cb_group_);
    fault_reset_client_ = create_client<std_srvs::srv::Trigger>(
      "/nav/fault/reset", rmw_qos_profile_services_default, client_cb_group_);

    loc_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav/loc_valid", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        loc_valid_ = msg->data;
        sm_.setSystemReady(loc_valid_ && !fault_active_);
      });

    fault_sub_ = create_subscription<tower_nav_msgs::msg::Fault>(
      "/nav/fault", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::Fault::ConstSharedPtr msg) {onFault(msg);});

    ref_sub_ = create_subscription<tower_nav_msgs::msg::TrackingReference>(
      "/nav/traj/reference", rclcpp::QoS(10),
      [this](tower_nav_msgs::msg::TrackingReference::ConstSharedPtr msg) {
        // 进度 100% 且跟踪不再激活 → 轨迹完成
        if (sm_.state() == MissionState::kExecuting &&
          msg->progress >= 100.0 && !msg->tracking_active)
        {
          sm_.notifyTrajFinished();
          RCLCPP_INFO(get_logger(), "任务完成:轨迹 %u", msg->traj_id);
        }
      });

    state_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "/nav/mission_state", rclcpp::QoS(10));

    cmd_srv_ = create_service<MissionSrv>(
      "/nav/mission_cmd",
      [this](const MissionSrv::Request::SharedPtr req,
      MissionSrv::Response::SharedPtr res) {onMissionCmd(req, res);},
      rmw_qos_profile_services_default, service_cb_group_);

    timer_ = create_wall_timer(100ms, [this]() {publishState();});

    RCLCPP_INFO(get_logger(), "任务管理节点已启动(对外入口 /nav/mission_cmd)");
  }

private:
  void onFault(tower_nav_msgs::msg::Fault::ConstSharedPtr msg)
  {
    const bool active = msg->active && msg->code != tower_nav_msgs::msg::Fault::NONE;
    if (active && !fault_active_) {
      RCLCPP_WARN(get_logger(), "任务收到故障:code=%u,进入 ERROR", msg->code);
      sm_.notifyFault();
      // 联动轨迹节点进入故障停止(先 PAUSE 再由 RESET 流程处理;
      // 简化:发送 STOP 使轨迹回 READY,任务保持 ERROR 直到复位)
      callTraj(TrajSrv::Request::STOP, 0);
    }
    fault_active_ = active;
    sm_.setSystemReady(loc_valid_ && !fault_active_);
  }

  void onMissionCmd(
    const MissionSrv::Request::SharedPtr req, MissionSrv::Response::SharedPtr res)
  {
    MissionCmd cmd;
    switch (req->command) {
      case MissionSrv::Request::START: cmd = MissionCmd::kStart; break;
      case MissionSrv::Request::PAUSE: cmd = MissionCmd::kPause; break;
      case MissionSrv::Request::RESUME: cmd = MissionCmd::kResume; break;
      case MissionSrv::Request::STOP: cmd = MissionCmd::kStop; break;
      case MissionSrv::Request::RESET: cmd = MissionCmd::kReset; break;
      default:
        res->success = false;
        res->message = "未知命令";
        res->mission_state = static_cast<uint8_t>(sm_.state());
        return;
    }

    // RESET 先尝试清除异常管理器故障
    if (cmd == MissionCmd::kReset && fault_active_) {
      if (!callFaultReset()) {
        res->success = false;
        res->message = "复位失败:故障未清除(急停可能仍激活)";
        res->mission_state = static_cast<uint8_t>(sm_.state());
        return;
      }
      fault_active_ = false;
      sm_.notifyFaultCleared();
      sm_.setSystemReady(loc_valid_);
    }

    // 状态机预检
    const CmdResult r = sm_.handle(cmd, req->traj_id);
    if (!r.success) {
      res->success = false;
      res->message = r.message;
      res->mission_state = static_cast<uint8_t>(sm_.state());
      return;
    }

    // 编排轨迹节点
    bool traj_ok = true;
    std::string traj_msg;
    switch (cmd) {
      case MissionCmd::kStart:
        traj_ok = callTraj(TrajSrv::Request::STOP, 0, &traj_msg, true) &&
          callTraj(TrajSrv::Request::LOAD, req->traj_id, &traj_msg) &&
          callTraj(TrajSrv::Request::START, 0, &traj_msg);
        break;
      case MissionCmd::kPause:
        traj_ok = callTraj(TrajSrv::Request::PAUSE, 0, &traj_msg);
        break;
      case MissionCmd::kResume:
        traj_ok = callTraj(TrajSrv::Request::RESUME, 0, &traj_msg);
        break;
      case MissionCmd::kStop:
        traj_ok = callTraj(TrajSrv::Request::STOP, 0, &traj_msg);
        break;
      case MissionCmd::kReset:
        callTraj(TrajSrv::Request::RESET, 0, &traj_msg, true);
        break;
    }

    if (!traj_ok) {
      // 轨迹层失败:回滚任务状态
      RCLCPP_ERROR(get_logger(), "轨迹编排失败: %s", traj_msg.c_str());
      if (cmd == MissionCmd::kStart) {
        sm_.handle(MissionCmd::kStop, 0);
      }
      res->success = false;
      res->message = "轨迹编排失败: " + traj_msg;
      res->mission_state = static_cast<uint8_t>(sm_.state());
      return;
    }

    RCLCPP_INFO(get_logger(), "%s", r.message.c_str());
    res->success = true;
    res->message = r.message;
    res->mission_state = static_cast<uint8_t>(sm_.state());
  }

  /// 同步调用轨迹服务;tolerate_fail=true 时失败不阻断流程
  bool callTraj(
    uint8_t command, uint8_t traj_id,
    std::string * msg_out = nullptr, bool tolerate_fail = false)
  {
    if (!traj_client_->wait_for_service(1s)) {
      if (msg_out) {*msg_out = "轨迹服务不可用";}
      return tolerate_fail;
    }
    auto req = std::make_shared<TrajSrv::Request>();
    req->command = command;
    req->traj_id = traj_id;
    auto future = traj_client_->async_send_request(req);
    if (future.wait_for(2s) != std::future_status::ready) {
      if (msg_out) {*msg_out = "轨迹服务超时";}
      return tolerate_fail;
    }
    const auto res = future.get();
    if (msg_out) {*msg_out = res->message;}
    return res->success || tolerate_fail;
  }

  bool callFaultReset()
  {
    if (!fault_reset_client_->wait_for_service(1s)) {
      return false;
    }
    auto future = fault_reset_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(2s) != std::future_status::ready) {
      return false;
    }
    return future.get()->success;
  }

  void publishState()
  {
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(sm_.state());
    state_pub_->publish(msg);
  }

  MissionStateMachine sm_;
  bool loc_valid_{false};
  bool fault_active_{false};

  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Client<TrajSrv>::SharedPtr traj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr fault_reset_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loc_valid_sub_;
  rclcpp::Subscription<tower_nav_msgs::msg::Fault>::SharedPtr fault_sub_;
  rclcpp::Subscription<tower_nav_msgs::msg::TrackingReference>::SharedPtr ref_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Service<MissionSrv>::SharedPtr cmd_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tower_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 服务回调中同步调用其它服务:用多线程执行器避免死锁
  auto node = std::make_shared<tower_nav::MissionNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
