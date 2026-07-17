// MissionStateMachine 单元测试:状态机与命令合法性迁移全覆盖。
#include <gtest/gtest.h>

#include "tower_nav/mission_state_machine.hpp"

namespace
{

using tower_nav::MissionCmd;
using tower_nav::MissionState;
using tower_nav::MissionStateMachine;

TEST(MissionStateMachine, InitialStandby)
{
  MissionStateMachine m;
  EXPECT_EQ(m.state(), MissionState::kStandby);
}

TEST(MissionStateMachine, StandbyToReadyOnSystemReady)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  EXPECT_EQ(m.state(), MissionState::kReady);
  m.setSystemReady(false);
  EXPECT_EQ(m.state(), MissionState::kStandby);
}

TEST(MissionStateMachine, StartRequiresReadyAndValidId)
{
  MissionStateMachine m;
  // STANDBY:拒绝
  EXPECT_FALSE(m.handle(MissionCmd::kStart, 1).success);
  m.setSystemReady(true);
  // 非法轨迹号
  EXPECT_FALSE(m.handle(MissionCmd::kStart, 0).success);
  EXPECT_FALSE(m.handle(MissionCmd::kStart, 5).success);
  // 合法启动
  const auto r = m.handle(MissionCmd::kStart, 2);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(m.state(), MissionState::kExecuting);
  EXPECT_EQ(m.activeTrajId(), 2);
}

TEST(MissionStateMachine, NoStartWhileExecuting)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  ASSERT_TRUE(m.handle(MissionCmd::kStart, 1).success);
  EXPECT_FALSE(m.handle(MissionCmd::kStart, 2).success);
  EXPECT_EQ(m.activeTrajId(), 1);
}

TEST(MissionStateMachine, PauseResumeFlow)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  ASSERT_TRUE(m.handle(MissionCmd::kStart, 1).success);
  // 执行中不可 RESUME
  EXPECT_FALSE(m.handle(MissionCmd::kResume, 0).success);
  EXPECT_TRUE(m.handle(MissionCmd::kPause, 0).success);
  EXPECT_EQ(m.state(), MissionState::kHolding);
  // 暂停中不可 PAUSE / START
  EXPECT_FALSE(m.handle(MissionCmd::kPause, 0).success);
  EXPECT_FALSE(m.handle(MissionCmd::kStart, 2).success);
  EXPECT_TRUE(m.handle(MissionCmd::kResume, 0).success);
  EXPECT_EQ(m.state(), MissionState::kExecuting);
}

TEST(MissionStateMachine, StopFromExecutingAndHolding)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  ASSERT_TRUE(m.handle(MissionCmd::kStart, 1).success);
  EXPECT_TRUE(m.handle(MissionCmd::kStop, 0).success);
  EXPECT_EQ(m.state(), MissionState::kAborted);
  EXPECT_EQ(m.activeTrajId(), 0);
  // ABORTED 后可重新启动
  EXPECT_TRUE(m.handle(MissionCmd::kStart, 3).success);
  ASSERT_TRUE(m.handle(MissionCmd::kPause, 0).success);
  EXPECT_TRUE(m.handle(MissionCmd::kStop, 0).success);
  EXPECT_EQ(m.state(), MissionState::kAborted);
}

TEST(MissionStateMachine, CompleteFlow)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  ASSERT_TRUE(m.handle(MissionCmd::kStart, 4).success);
  m.notifyTrajFinished();
  EXPECT_EQ(m.state(), MissionState::kCompleted);
  // 完成后可直接启动新轨迹
  EXPECT_TRUE(m.handle(MissionCmd::kStart, 2).success);
  EXPECT_EQ(m.activeTrajId(), 2);
}

TEST(MissionStateMachine, FaultFlow)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  ASSERT_TRUE(m.handle(MissionCmd::kStart, 1).success);
  m.notifyFault();
  EXPECT_EQ(m.state(), MissionState::kError);
  // ERROR 态拒绝 START/PAUSE/RESUME/STOP
  EXPECT_FALSE(m.handle(MissionCmd::kStart, 1).success);
  EXPECT_FALSE(m.handle(MissionCmd::kPause, 0).success);
  EXPECT_FALSE(m.handle(MissionCmd::kResume, 0).success);
  EXPECT_FALSE(m.handle(MissionCmd::kStop, 0).success);
  // RESET 恢复
  EXPECT_TRUE(m.handle(MissionCmd::kReset, 0).success);
  EXPECT_EQ(m.state(), MissionState::kReady);
  EXPECT_TRUE(m.handle(MissionCmd::kStart, 1).success);
}

TEST(MissionStateMachine, FaultClearedNotification)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  m.notifyFault();
  EXPECT_EQ(m.state(), MissionState::kError);
  m.notifyFaultCleared();
  EXPECT_EQ(m.state(), MissionState::kReady);
}

TEST(MissionStateMachine, ResetFromAbortedAndCompleted)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  ASSERT_TRUE(m.handle(MissionCmd::kStart, 1).success);
  ASSERT_TRUE(m.handle(MissionCmd::kStop, 0).success);
  EXPECT_TRUE(m.handle(MissionCmd::kReset, 0).success);
  EXPECT_EQ(m.state(), MissionState::kReady);

  ASSERT_TRUE(m.handle(MissionCmd::kStart, 1).success);
  m.notifyTrajFinished();
  EXPECT_TRUE(m.handle(MissionCmd::kReset, 0).success);
  EXPECT_EQ(m.state(), MissionState::kReady);
}

TEST(MissionStateMachine, ResetIdempotentFromReadyAndStandby)
{
  // 故障清除流程可能已把状态迁回 READY/STANDBY,此时 RESET 应幂等成功
  MissionStateMachine m;
  EXPECT_TRUE(m.handle(MissionCmd::kReset, 0).success);   // STANDBY
  EXPECT_EQ(m.state(), MissionState::kStandby);
  m.setSystemReady(true);
  EXPECT_TRUE(m.handle(MissionCmd::kReset, 0).success);   // READY
  EXPECT_EQ(m.state(), MissionState::kReady);
}

TEST(MissionStateMachine, ResetInvalidWhileExecuting)
{
  MissionStateMachine m;
  m.setSystemReady(true);
  ASSERT_TRUE(m.handle(MissionCmd::kStart, 1).success);
  EXPECT_FALSE(m.handle(MissionCmd::kReset, 0).success);
}

}  // namespace
