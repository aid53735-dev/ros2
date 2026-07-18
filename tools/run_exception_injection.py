#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T19 异常注入闭环验证。

一次拉起全栈,依次注入三类异常并验证处置与恢复:
  1. 停发 IMU(/sim/imu_enable false)→ IMU_TIMEOUT(2)+ 安全停车 → 恢复 IMU
     → RESET → 重新 START 可作业;
  2. 横向偏差超阈:以高频对抗性 /cmd_vel(模拟外力扰动,ω 超控制器限幅)把
     机器人推离轨迹,估计位姿的横向误差真实超阈 → LATERAL_DEV(6)+ 安全停车
     → RESET → 恢复作业;
  3. 急停 /nav/estop true → ESTOP(7)+ /cmd_vel=0;急停未解除时 RESET 必须失败;
     解除急停 → RESET → 重新 START 恢复作业。

每步记录实测:故障码、/cmd_vel 是否归零、复位与恢复结果。输出 JSON。

用法:run_exception_injection.py <输出.json>
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import SetBool
from tower_nav_msgs.msg import Fault, TrackingReference
from tower_nav_msgs.srv import MissionCommand

FAULT_IMU_TIMEOUT = 2
FAULT_LATERAL_DEV = 6
FAULT_ESTOP = 7
MISSION_EXECUTING = 2
MISSION_ERROR = 6


class Runner:
    """异常注入验证运行器(复用全栈,一次拉起)。"""

    def __init__(self):
        self.node = rclpy.create_node("t19_runner")
        self.mission_state = None
        self.fault_code = 0
        self.fault_active = False
        self.cmd_vels = []
        self.refs = []
        self.pose = None  # (x, y, yaw) 定位估计
        self.node.create_subscription(
            UInt8, "/nav/mission_state",
            lambda m: setattr(self, "mission_state", m.data), 10)
        self.node.create_subscription(Fault, "/nav/fault", self._on_fault, 10)
        self.node.create_subscription(Twist, "/cmd_vel", self.cmd_vels.append, 50)
        self.node.create_subscription(
            TrackingReference, "/nav/traj/reference", self.refs.append, 50)
        self.node.create_subscription(
            PoseWithCovarianceStamped, "/nav/pose", self._on_pose, 10)
        self.estop_pub = self.node.create_publisher(Bool, "/nav/estop", 10)
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.mission_cli = self.node.create_client(MissionCommand, "/nav/mission_cmd")
        self.imu_cli = self.node.create_client(SetBool, "/sim/imu_enable")
        self.slip_cli = self.node.create_client(SetBool, "/sim/set_slip")

    def _on_fault(self, m):
        self.fault_code = m.code
        self.fault_active = m.active

    def _on_pose(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def destroy(self):
        self.node.destroy_node()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def mission(self, command, traj_id=0):
        req = MissionCommand.Request()
        req.command = command
        req.traj_id = traj_id
        res = self._call(self.mission_cli, req)
        if res is None:
            return False, "服务不可用/超时"
        return res.success, res.message

    def mission_retry(self, command, traj_id=0, attempts=15, delay=1.0):
        msg = ""
        for _ in range(attempts):
            ok, msg = self.mission(command, traj_id)
            if ok:
                return True, msg
            self.spin_for(delay)
        return False, msg

    def set_bool(self, cli, value):
        req = SetBool.Request()
        req.data = value
        res = self._call(cli, req)
        return res is not None and res.success

    def publish_estop(self, value):
        msg = Bool()
        msg.data = value
        self.estop_pub.publish(msg)

    def wait_fault(self, codes, timeout):
        """等待任一目标故障码激活;codes 可为 int 或集合。"""
        if isinstance(codes, int):
            codes = {codes}
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.fault_active and self.fault_code in codes:
                return True
        return False

    def wait_mission_state(self, state, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.mission_state == state:
                return True
        return False

    def wait_tracking(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.refs and self.refs[-1].tracking_active and \
                    self.refs[-1].progress > 0.5:
                return True
        return False

    def measure_cmd_zero(self, settle=1.0, window=1.5):
        """settle 后采样 window 秒,返回 (vmax, wmax)。"""
        self.spin_for(settle)
        self.cmd_vels.clear()
        self.spin_for(window)
        vmax = max((abs(m.linear.x) for m in self.cmd_vels), default=0.0)
        wmax = max((abs(m.angular.z) for m in self.cmd_vels), default=0.0)
        return vmax, wmax

    def start_and_track(self, traj_id=1):
        """START 并等待跟踪推进(作为每个场景的前置)。"""
        ok, msg = self.mission_retry(MissionCommand.Request.START, traj_id)
        assert ok, f"START 失败: {msg}"
        assert self.wait_tracking(60.0), "轨迹未开始推进"

    def reset_and_verify_recovery(self, rec, key):
        """RESET → 重新 START 验证恢复作业 → STOP 清场。"""
        ok, msg = self.mission_retry(MissionCommand.Request.RESET, attempts=10)
        assert ok, f"RESET 失败: {msg}"
        self.spin_for(1.0)
        assert not self.fault_active, f"RESET 后故障未清除 code={self.fault_code}"
        self.refs.clear()
        self.start_and_track()
        rec["checks"][key] = "RESET 后重新 START,跟踪恢复推进"
        ok, msg = self.mission(MissionCommand.Request.STOP)
        assert ok, f"清场 STOP 失败: {msg}"
        self.spin_for(1.0)

    def pilot_to_line(self, target_y, target_yaw=0.0, tol=0.05, timeout=90.0):
        """手动遥控把机器人开回 y=target_y 直线并对准 target_yaw。

        用于横向偏差场景后的复位:机器人已被扰动推离轨迹,直接重新 START
        会立即再次超阈;先驾回轨迹带再恢复作业(等效现场人工复位归位)。
        高频发布以覆盖控制器的 50 Hz 零速指令。
        """
        end = time.monotonic() + timeout
        cmd = Twist()
        # 第一阶段:回到目标直线
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.002)
            if self.pose is None:
                continue
            _, y, yaw = self.pose
            dy = target_y - y
            if abs(dy) < tol:
                break
            desired = math.pi / 2 if dy > 0 else -math.pi / 2
            err = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * err))
            cmd.linear.x = 0.12 if abs(err) < 0.6 else 0.0
            self.cmd_pub.publish(cmd)
        # 第二阶段:原地对准轨迹切向
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.002)
            if self.pose is None:
                continue
            _, y, yaw = self.pose
            err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
            if abs(err) < 0.08 and abs(target_y - y) < 2 * tol:
                self.cmd_pub.publish(Twist())
                return True
            cmd.linear.x = 0.0
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * err))
            self.cmd_pub.publish(cmd)
        self.cmd_pub.publish(Twist())
        return False


def scenario_imu(r, rec):
    """场景 1:停发 IMU → IMU_TIMEOUT + 安全停车 → 恢复 → 复位续作业。"""
    r.start_and_track()
    assert r.set_bool(r.imu_cli, False), "/sim/imu_enable false 调用失败"
    assert r.wait_fault(FAULT_IMU_TIMEOUT, 15.0), \
        f"未触发 IMU_TIMEOUT(当前 code={r.fault_code})"
    rec["checks"]["fault_code"] = f"IMU 停发 → 故障码 {r.fault_code}(IMU_TIMEOUT=2)"
    assert r.wait_mission_state(MISSION_ERROR, 10.0), "任务未进入 ERROR"
    rec["checks"]["mission_error"] = "任务状态进入 ERROR(6)"
    vmax, wmax = r.measure_cmd_zero()
    assert vmax < 1e-6 and wmax < 1e-6, f"安全停车失败 v={vmax} w={wmax}"
    rec["checks"]["safe_stop"] = f"安全停车 |v|max={vmax:.2e} |w|max={wmax:.2e}"
    # 恢复 IMU 后复位
    assert r.set_bool(r.imu_cli, True), "/sim/imu_enable true 调用失败"
    r.spin_for(2.0)
    r.reset_and_verify_recovery(rec, "recovery")


def scenario_lateral(r, rec):
    """场景 2:对抗性扰动推离轨迹 → 横向偏差超阈 → LATERAL_DEV + 安全停车。"""
    r.start_and_track()
    # 高频发布扰动 /cmd_vel(高于控制器 50 Hz,仿真按最新指令执行),
    # 模拟外力把机器人推离轨迹;定位如实反映 → 横向误差超阈持续 → 触发故障
    end = time.monotonic() + 60.0
    disturb = Twist()
    disturb.linear.x = 0.6
    disturb.angular.z = 1.5  # 回转半径 0.4 m,足以把 |e_y| 推过 0.30 m 阈值
    triggered = False
    while time.monotonic() < end:
        r.cmd_pub.publish(disturb)
        rclpy.spin_once(r.node, timeout_sec=0.002)
        if r.fault_active and r.fault_code == FAULT_LATERAL_DEV:
            triggered = True
            break
    last_err = abs(r.refs[-1].lateral_error) if r.refs else float("nan")
    assert triggered, \
        f"未触发 LATERAL_DEV(当前 code={r.fault_code},|e_y|={last_err:.3f})"
    rec["checks"]["fault_code"] = \
        f"扰动推离 → 故障码 6(LATERAL_DEV),触发时 |e_y|={last_err:.3f} m"
    assert r.wait_mission_state(MISSION_ERROR, 10.0), "任务未进入 ERROR"
    rec["checks"]["mission_error"] = "任务状态进入 ERROR(6)"
    vmax, wmax = r.measure_cmd_zero()
    assert vmax < 1e-6 and wmax < 1e-6, f"安全停车失败 v={vmax} w={wmax}"
    rec["checks"]["safe_stop"] = f"安全停车 |v|max={vmax:.2e} |w|max={wmax:.2e}"
    # 复位后先人工归位(机器人已被扰动推离轨迹,直接重跑会立即再超阈),
    # 再重新 START 验证恢复作业
    ok, msg = r.mission_retry(MissionCommand.Request.RESET, attempts=10)
    assert ok, f"RESET 失败: {msg}"
    r.spin_for(1.0)
    assert not r.fault_active, f"RESET 后故障未清除 code={r.fault_code}"
    assert r.pilot_to_line(20.0, 0.0), "人工归位失败(未回到轨迹带)"
    r.refs.clear()
    r.start_and_track()
    rec["checks"]["recovery"] = "RESET + 归位后重新 START,跟踪恢复推进"
    ok, msg = r.mission(MissionCommand.Request.STOP)
    assert ok, f"清场 STOP 失败: {msg}"
    r.spin_for(1.0)


def scenario_estop(r, rec):
    """场景 3:急停 → ESTOP + 零速;未解除时 RESET 拒绝;解除后恢复。"""
    r.start_and_track()
    r.publish_estop(True)
    assert r.wait_fault(FAULT_ESTOP, 10.0), \
        f"未触发 ESTOP(当前 code={r.fault_code})"
    rec["checks"]["fault_code"] = f"急停 → 故障码 {r.fault_code}(ESTOP=7)"
    assert r.wait_mission_state(MISSION_ERROR, 10.0), "任务未进入 ERROR"
    vmax, wmax = r.measure_cmd_zero()
    assert vmax < 1e-6 and wmax < 1e-6, f"急停停车失败 v={vmax} w={wmax}"
    rec["checks"]["safe_stop"] = f"急停零速 |v|max={vmax:.2e} |w|max={wmax:.2e}"
    # 急停未解除:RESET 必须被拒
    ok, msg = r.mission(MissionCommand.Request.RESET)
    assert not ok, "急停未解除时 RESET 未被拒绝"
    rec["checks"]["reset_guard"] = f"急停未解除 RESET 被拒: {msg}"
    # 解除急停 → 复位恢复
    r.publish_estop(False)
    r.spin_for(1.0)
    r.reset_and_verify_recovery(rec, "recovery")


def main():
    out_path = sys.argv[1]
    launch = subprocess.Popen(
        ["ros2", "launch", "tower_nav", "bringup.launch.py"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    rclpy.init()
    r = Runner()
    scenarios = [
        ("imu_stop", "停发 IMU", scenario_imu),
        ("lateral_dev", "横向偏差超阈(扰动推离)", scenario_lateral),
        ("estop", "急停触发与保护", scenario_estop),
    ]
    results = []
    try:
        r.spin_for(6.0)  # 各节点上线 + 启动宽限期
        for key, desc, fn in scenarios:
            rec = {"scenario": key, "description": desc,
                   "checks": {}, "passed": False}
            print(f"=== 场景 {key}: {desc} ===", flush=True)
            try:
                fn(r, rec)
                rec["passed"] = True
            except AssertionError as exc:
                rec["error"] = str(exc)
            results.append(rec)
            print(json.dumps(rec, ensure_ascii=False, indent=2), flush=True)
    finally:
        r.destroy()
        rclpy.shutdown()
        os.killpg(os.getpgid(launch.pid), signal.SIGINT)
        try:
            launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    n_pass = sum(1 for x in results if x["passed"])
    print(f"通过 {n_pass}/{len(results)}", flush=True)
    sys.exit(0 if n_pass == len(results) else 1)


if __name__ == "__main__":
    main()
