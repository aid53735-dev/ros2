#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T17 四条轨迹全流程闭环验证。

对每条轨迹:
  1. 以该轨迹起点作为初始位姿拉起全栈(bringup.launch.py);
  2. START → (轨迹1 额外验证 STOP/重启 与 RUNNING 中 START 被拒) → PAUSE →
     检查暂停零速 → RESUME → 跟踪至 COMPLETED;
  3. 记录最大横向误差、完成用时等实测数据,输出 JSON。

用法:run_all_trajectories.py <输出.json> [traj_ids...] [--traj-dir DIR]
     --traj-dir 指定备选轨迹目录(T18 塔筒参数更换验证,零代码修改)。
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8
from tower_nav_msgs.msg import Fault, TrackingReference
from tower_nav_msgs.srv import MissionCommand

MISSION_COMPLETED = 4
V_NOMINAL = 0.15  # 轨迹标称速度 m/s(与 gen_trajectories.py 一致)


def traj_meta(traj_dir, traj_id):
    """读取轨迹文件起点与总长度(起点位姿用于初始位姿参数)。"""
    with open(os.path.join(traj_dir, f"traj_{traj_id}.yaml"), encoding="utf-8") as f:
        data = yaml.safe_load(f)["trajectory"]
    pts = data["points"]
    length = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        length += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    p0 = pts[0]
    return {"x0": p0["x"], "y0": p0["y"], "yaw0": p0["yaw"],
            "length": length, "n_points": len(pts),
            "description": data.get("description", "")}


class Runner:
    """单条轨迹的闭环运行器。"""

    def __init__(self, traj_id):
        self.node = rclpy.create_node(f"t17_runner_{traj_id}")
        self.refs = []
        self.mission_state = None
        self.loc_valid = False
        self.cmd_vels = []
        self.fault_code = 0
        self.node.create_subscription(
            TrackingReference, "/nav/traj/reference", self.refs.append, 50)
        self.node.create_subscription(
            UInt8, "/nav/mission_state",
            lambda m: setattr(self, "mission_state", m.data), 10)
        self.node.create_subscription(
            Bool, "/nav/loc_valid",
            lambda m: setattr(self, "loc_valid", m.data), 10)
        self.node.create_subscription(
            Twist, "/cmd_vel", self.cmd_vels.append, 50)
        self.node.create_subscription(
            Fault, "/nav/fault",
            lambda m: setattr(self, "fault_code", m.code), 10)
        self.cli = self.node.create_client(MissionCommand, "/nav/mission_cmd")

    def destroy(self):
        self.node.destroy_node()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call(self, command, traj_id=0, timeout=10.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False, "服务不可用"
        req = MissionCommand.Request()
        req.command = command
        req.traj_id = traj_id
        fut = self.cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not fut.done():
            return False, "调用超时"
        res = fut.result()
        return res.success, res.message

    def call_retry(self, command, traj_id=0, attempts=15, delay=1.0):
        msg = ""
        for _ in range(attempts):
            ok, msg = self.call(command, traj_id)
            if ok:
                return True, msg
            self.spin_for(delay)
        return False, msg

    def wait_progress(self, threshold, timeout):
        """自旋直到跟踪进度超过 threshold(%)。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.refs and self.refs[-1].tracking_active and \
                    self.refs[-1].progress >= threshold:
                return True
        return False

    def wait_mission_state(self, state, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.mission_state == state:
                return True
        return False


def run_one(traj_id, meta, extra_checks, traj_dir=None):
    """运行一条轨迹,返回实测记录 dict。"""
    rec = {"traj_id": traj_id, "description": meta["description"],
           "length_m": round(meta["length"], 3), "n_points": meta["n_points"],
           "checks": {}, "passed": False}
    cmd = ["ros2", "launch", "tower_nav", "bringup.launch.py",
           f"initial_x:={meta['x0']}", f"initial_y:={meta['y0']}",
           f"initial_yaw:={meta['yaw0']}"]
    if traj_dir:
        cmd.append(f"trajectory_dir:={traj_dir}")
    launch = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    r = Runner(traj_id)
    t_start = time.monotonic()
    try:
        r.spin_for(6.0)  # 等各节点上线 + 启动宽限期

        ok, msg = r.call_retry(MissionCommand.Request.START, traj_id)
        assert ok, f"START 失败: {msg}"

        if extra_checks:
            # RUNNING 中 START 其它轨迹必须被拒(切换保护)
            assert r.wait_progress(2.0, 60.0), "轨迹未开始推进"
            ok, msg = r.call(MissionCommand.Request.START, 2)
            assert not ok, "RUNNING 中 START 未被拒绝"
            rec["checks"]["switch_guard"] = f"RUNNING 中 START 被拒: {msg}"
            # STOP → ABORTED → 重新 START(进度清零重跑)
            ok, msg = r.call(MissionCommand.Request.STOP)
            assert ok, f"STOP 失败: {msg}"
            assert r.wait_mission_state(5, 10.0), "STOP 后未进入 ABORTED"
            rec["checks"]["stop"] = "STOP → ABORTED 正常"
            ok, msg = r.call_retry(MissionCommand.Request.START, traj_id)
            assert ok, f"STOP 后重新 START 失败: {msg}"
            rec["checks"]["restart_after_stop"] = "ABORTED 后重新 START 正常"

        # 跟踪到 25% 后 PAUSE
        est_time = meta["length"] / V_NOMINAL
        assert r.wait_progress(25.0, est_time), "未达到 25% 进度"
        ok, msg = r.call(MissionCommand.Request.PAUSE)
        assert ok, f"PAUSE 失败: {msg}"
        r.spin_for(3.0)  # 等减速完成
        r.cmd_vels.clear()
        r.spin_for(1.5)
        vmax = max((abs(m.linear.x) for m in r.cmd_vels), default=0.0)
        wmax = max((abs(m.angular.z) for m in r.cmd_vels), default=0.0)
        assert vmax < 1e-6 and wmax < 1e-6, f"暂停期间速度非零 v={vmax} w={wmax}"
        rec["checks"]["pause_zero_vel"] = \
            f"暂停期间 |v|max={vmax:.2e} |w|max={wmax:.2e}"
        progress_at_pause = r.refs[-1].progress if r.refs else 0.0

        # RESUME 断点续跑至完成
        ok, msg = r.call(MissionCommand.Request.RESUME)
        assert ok, f"RESUME 失败: {msg}"
        r.refs.clear()
        assert r.wait_progress(progress_at_pause + 1.0, 120.0), "RESUME 后进度未增长"
        rec["checks"]["resume"] = f"断点 {progress_at_pause:.1f}% 恢复后继续推进"

        timeout = est_time * 1.5 + 120.0
        assert r.wait_mission_state(MISSION_COMPLETED, timeout), \
            f"超时未完成(fault_code={r.fault_code}, state={r.mission_state})"
        elapsed = time.monotonic() - t_start

        errs = [abs(m.lateral_error) for m in r.refs if m.tracking_active]
        rec["max_lateral_error_m"] = round(max(errs), 4) if errs else None
        rec["elapsed_s"] = round(elapsed, 1)
        rec["final_fault_code"] = r.fault_code
        rec["passed"] = True
    except AssertionError as exc:
        rec["error"] = str(exc)
    finally:
        r.destroy()
        os.killpg(os.getpgid(launch.pid), signal.SIGINT)
        try:
            launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
        time.sleep(2.0)  # 等 DDS 资源释放
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_path")
    ap.add_argument("ids", nargs="*", type=int)
    ap.add_argument("--traj-dir", default=None,
                    help="备选轨迹目录(默认用包内 config/trajectories)")
    args = ap.parse_args()
    out_path = args.out_path
    ids = args.ids or [1, 2, 3, 4]
    if args.traj_dir:
        traj_dir = args.traj_dir
    else:
        from ament_index_python.packages import get_package_share_directory
        traj_dir = os.path.join(
            get_package_share_directory("tower_nav"), "config", "trajectories")

    rclpy.init()
    results = []
    for tid in ids:
        meta = traj_meta(traj_dir, tid)
        print(f"=== 轨迹 {tid}: {meta['description']} "
              f"(长度 {meta['length']:.1f} m)===", flush=True)
        rec = run_one(tid, meta, extra_checks=(tid == ids[0]),
                      traj_dir=args.traj_dir)
        results.append(rec)
        print(json.dumps(rec, ensure_ascii=False, indent=2), flush=True)
    rclpy.shutdown()

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    n_pass = sum(1 for x in results if x["passed"])
    print(f"通过 {n_pass}/{len(results)}", flush=True)
    sys.exit(0 if n_pass == len(results) else 1)


if __name__ == "__main__":
    main()
