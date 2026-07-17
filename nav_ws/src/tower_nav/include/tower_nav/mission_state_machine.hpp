// 任务管理核心(纯逻辑,可单测):
// 状态机 STANDBY/READY/EXECUTING/HOLDING/COMPLETED/ABORTED/ERROR;
// 处理 START/PAUSE/RESUME/STOP/RESET 命令的合法性与迁移。
#pragma once

#include <cstdint>
#include <string>

namespace tower_nav
{

/// 任务状态(与 tower_nav_msgs/NavStatus MISSION_* 常量一致)
enum class MissionState : uint8_t
{
  kStandby = 0,     // 待机:系统未就绪(定位无效等)
  kReady = 1,       // 就绪:可接受 START
  kExecuting = 2,   // 执行中
  kHolding = 3,     // 暂停(安全驻停)
  kCompleted = 4,   // 轨迹完成
  kAborted = 5,     // 用户停止
  kError = 6,       // 故障
};

const char * toString(MissionState s);

/// 任务命令(与 MissionCommand.srv 常量一致)
enum class MissionCmd : uint8_t
{
  kStart = 0,
  kPause = 1,
  kResume = 2,
  kStop = 3,
  kReset = 4,
};

/// 命令处理结果
struct CmdResult
{
  bool success{false};
  std::string message;
};

/// 任务状态机
class MissionStateMachine
{
public:
  MissionStateMachine() = default;

  /// 系统就绪信号(定位有效且轨迹节点在线):STANDBY↔READY 自动迁移
  void setSystemReady(bool ready);

  /// 处理命令。traj_id 仅 START 用(1~4)。
  /// 返回结果;成功时状态已迁移。调用方负责联动轨迹/控制器。
  CmdResult handle(MissionCmd cmd, uint8_t traj_id);

  /// 轨迹完成通知(EXECUTING→COMPLETED)
  void notifyTrajFinished();

  /// 故障通知(任意态→ERROR)
  void notifyFault();

  /// 故障清除通知(ERROR→STANDBY,随后由 setSystemReady 恢复)
  void notifyFaultCleared();

  MissionState state() const {return state_;}
  uint8_t activeTrajId() const {return active_traj_;}

private:
  MissionState state_{MissionState::kStandby};
  uint8_t active_traj_{0};
  bool system_ready_{false};
};

}  // namespace tower_nav
