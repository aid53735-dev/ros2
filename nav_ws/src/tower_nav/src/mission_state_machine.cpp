// 任务状态机实现。
#include "tower_nav/mission_state_machine.hpp"

namespace tower_nav
{

const char * toString(MissionState s)
{
  switch (s) {
    case MissionState::kStandby: return "STANDBY";
    case MissionState::kReady: return "READY";
    case MissionState::kExecuting: return "EXECUTING";
    case MissionState::kHolding: return "HOLDING";
    case MissionState::kCompleted: return "COMPLETED";
    case MissionState::kAborted: return "ABORTED";
    case MissionState::kError: return "ERROR";
  }
  return "UNKNOWN";
}

void MissionStateMachine::setSystemReady(bool ready)
{
  system_ready_ = ready;
  // STANDBY 且系统就绪 → READY;READY 而系统失去就绪 → STANDBY
  if (state_ == MissionState::kStandby && ready) {
    state_ = MissionState::kReady;
  } else if (state_ == MissionState::kReady && !ready) {
    state_ = MissionState::kStandby;
  }
  // COMPLETED / ABORTED 视为空闲态,可随就绪信号回 READY/STANDBY
  if ((state_ == MissionState::kCompleted || state_ == MissionState::kAborted) && ready) {
    // 保持完成/中止显示,直到下一条命令;不自动迁移
  }
}

CmdResult MissionStateMachine::handle(MissionCmd cmd, uint8_t traj_id)
{
  CmdResult r;
  switch (cmd) {
    case MissionCmd::kStart:
      // READY / COMPLETED / ABORTED 且系统就绪时允许启动
      if ((state_ == MissionState::kReady || state_ == MissionState::kCompleted ||
        state_ == MissionState::kAborted) && system_ready_)
      {
        if (traj_id < 1 || traj_id > 4) {
          r.message = "轨迹编号必须为 1~4";
          return r;
        }
        active_traj_ = traj_id;
        state_ = MissionState::kExecuting;
        r.success = true;
        r.message = "任务启动:轨迹 " + std::to_string(traj_id);
      } else {
        r.message = std::string("START 非法:当前状态 ") + toString(state_) +
          (system_ready_ ? "" : "(系统未就绪)");
      }
      return r;

    case MissionCmd::kPause:
      if (state_ == MissionState::kExecuting) {
        state_ = MissionState::kHolding;
        r.success = true;
        r.message = "任务暂停(安全驻停)";
      } else {
        r.message = std::string("PAUSE 非法:当前状态 ") + toString(state_);
      }
      return r;

    case MissionCmd::kResume:
      if (state_ == MissionState::kHolding) {
        state_ = MissionState::kExecuting;
        r.success = true;
        r.message = "任务恢复(断点续跑)";
      } else {
        r.message = std::string("RESUME 非法:当前状态 ") + toString(state_);
      }
      return r;

    case MissionCmd::kStop:
      if (state_ == MissionState::kExecuting || state_ == MissionState::kHolding) {
        state_ = MissionState::kAborted;
        active_traj_ = 0;
        r.success = true;
        r.message = "任务停止";
      } else {
        r.message = std::string("STOP 非法:当前状态 ") + toString(state_);
      }
      return r;

    case MissionCmd::kReset:
      if (state_ == MissionState::kError || state_ == MissionState::kAborted ||
        state_ == MissionState::kCompleted)
      {
        state_ = system_ready_ ? MissionState::kReady : MissionState::kStandby;
        active_traj_ = 0;
        r.success = true;
        r.message = "任务复位";
      } else if (state_ == MissionState::kReady || state_ == MissionState::kStandby) {
        // 幂等复位:故障清除流程可能已把状态迁回 READY/STANDBY,再次 RESET 视为成功
        active_traj_ = 0;
        r.success = true;
        r.message = "任务复位(系统已就绪)";
      } else {
        r.message = std::string("RESET 非法:当前状态 ") + toString(state_);
      }
      return r;
  }
  r.message = "未知命令";
  return r;
}

void MissionStateMachine::notifyTrajFinished()
{
  if (state_ == MissionState::kExecuting) {
    state_ = MissionState::kCompleted;
  }
}

void MissionStateMachine::notifyFault()
{
  state_ = MissionState::kError;
}

void MissionStateMachine::notifyFaultCleared()
{
  if (state_ == MissionState::kError) {
    state_ = system_ready_ ? MissionState::kReady : MissionState::kStandby;
    active_traj_ = 0;
  }
}

}  // namespace tower_nav
