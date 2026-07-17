#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""launch_testing 集成测试:sim + 全栈闭环。

流程:启动 sim + 全部节点 → START 轨迹 1 → 断言:
  1. 跟踪期间最大横向误差 < 阈值;
  2. PAUSE 后速度为零;
  3. RESUME 后继续推进(进度增长);
  4. 急停后 /cmd_vel = 0 且上报 ESTOP 故障码。
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tower_nav_msgs.msg import Fault, TrackingReference
from tower_nav_msgs.srv import MissionCommand

LATERAL_ERROR_LIMIT = 0.15  # m,集成验收阈值


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory("tower_nav")
    sim_share = get_package_share_directory("tower_nav_sim")
    config = os.path.join(pkg_share, "config")
    traj_dir = os.path.join(config, "trajectories")

    # 初始位姿设在轨迹 1 起点(定高环向:h=20 m,x=0,yaw=0)
    nodes = [
        launch_ros.actions.Node(
            package="tower_nav_sim", executable="sim_node", name="tower_nav_sim",
            parameters=[os.path.join(sim_share, "config", "sim.yaml")],
        ),
        launch_ros.actions.Node(
            package="tower_nav", executable="localization_node", name="localization_node",
            parameters=[
                os.path.join(config, "localization.yaml"),
                {"initial_y": 20.0},
            ],
        ),
        launch_ros.actions.Node(
            package="tower_nav", executable="trajectory_node", name="trajectory_node",
            parameters=[
                os.path.join(config, "mission.yaml"),
                {"trajectory_dir": traj_dir},
            ],
        ),
        launch_ros.actions.Node(
            package="tower_nav", executable="controller_node", name="controller_node",
            parameters=[os.path.join(config, "control.yaml")],
        ),
        launch_ros.actions.Node(
            package="tower_nav", executable="exception_node", name="exception_node",
            parameters=[os.path.join(config, "mission.yaml")],
        ),
        launch_ros.actions.Node(
            package="tower_nav", executable="mission_node", name="mission_node"),
        launch_ros.actions.Node(
            package="tower_nav", executable="status_node", name="status_node"),
    ]
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()])


class TestMissionFlow(unittest.TestCase):
    """全栈闭环集成测试。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("integration_tester")
        cls.cmd_vels = []
        cls.refs = []
        cls.faults = []
        cls.node.create_subscription(
            Twist, "/cmd_vel", lambda m: cls.cmd_vels.append(m), 50)
        cls.node.create_subscription(
            TrackingReference, "/nav/traj/reference", lambda m: cls.refs.append(m), 50)
        cls.node.create_subscription(
            Fault, "/nav/fault", lambda m: cls.faults.append(m), 50)
        cls.estop_pub = cls.node.create_publisher(Bool, "/nav/estop", 10)
        cls.mission_cli = cls.node.create_client(MissionCommand, "/nav/mission_cmd")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call_mission(self, command, traj_id=0, expect_success=True):
        self.assertTrue(
            self.mission_cli.wait_for_service(timeout_sec=10.0), "mission 服务不可用")
        req = MissionCommand.Request()
        req.command = command
        req.traj_id = traj_id
        future = self.mission_cli.call_async(req)
        end = time.time() + 10.0
        while not future.done() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(future.done(), "mission 服务调用超时")
        res = future.result()
        if expect_success:
            self.assertTrue(res.success, f"命令 {command} 失败: {res.message}")
        return res

    def call_mission_retry(self, command, traj_id=0, attempts=10, delay=1.0):
        """带重试的任务命令调用:系统就绪存在启动时序差,重试直到成功。"""
        last = None
        for _ in range(attempts):
            last = self.call_mission(command, traj_id, expect_success=False)
            if last.success:
                return last
            self.spin_for(delay)
        self.fail(f"命令 {command} 重试后仍失败: {last.message}")

    def test_full_flow(self):
        """启动→跟踪→暂停→恢复→急停 全流程断言。"""
        # 等系统就绪(定位有效、服务上线、启动宽限期过)
        self.spin_for(6.0)

        # ---- 1. START 轨迹 1,跟踪一段时间,检查横向误差 ----
        self.call_mission_retry(MissionCommand.Request.START, 1)
        self.refs.clear()
        self.spin_for(15.0)
        active_refs = [r for r in self.refs if r.tracking_active]
        self.assertGreater(len(active_refs), 50, "跟踪参考未持续发布")
        max_lat = max(abs(r.lateral_error) for r in active_refs)
        self.assertLess(
            max_lat, LATERAL_ERROR_LIMIT,
            f"最大横向误差 {max_lat:.3f} m 超过阈值 {LATERAL_ERROR_LIMIT} m")
        progress_before_pause = active_refs[-1].progress
        self.assertGreater(progress_before_pause, 0.0, "轨迹无进度")

        # ---- 2. PAUSE:速度应降为零 ----
        self.call_mission(MissionCommand.Request.PAUSE)
        self.spin_for(3.0)   # 等减速完成
        self.cmd_vels.clear()
        self.spin_for(2.0)
        self.assertGreater(len(self.cmd_vels), 10, "暂停期间 /cmd_vel 未发布")
        max_v = max(abs(m.linear.x) for m in self.cmd_vels)
        max_w = max(abs(m.angular.z) for m in self.cmd_vels)
        self.assertLess(max_v, 1e-6, f"PAUSE 后线速度 {max_v} 非零")
        self.assertLess(max_w, 1e-6, f"PAUSE 后角速度 {max_w} 非零")

        # ---- 3. RESUME:进度继续增长 ----
        self.call_mission(MissionCommand.Request.RESUME)
        self.refs.clear()
        self.spin_for(8.0)
        active_refs = [r for r in self.refs if r.tracking_active]
        self.assertGreater(len(active_refs), 10, "恢复后无跟踪参考")
        self.assertGreater(
            active_refs[-1].progress, progress_before_pause,
            "RESUME 后进度未增长(断点恢复失败)")

        # ---- 4. 急停:/cmd_vel=0 且上报 ESTOP 故障码 ----
        estop = Bool()
        estop.data = True
        for _ in range(5):
            self.estop_pub.publish(estop)
            self.spin_for(0.1)
        self.spin_for(1.0)
        self.cmd_vels.clear()
        self.faults.clear()
        self.spin_for(2.0)
        self.assertGreater(len(self.cmd_vels), 10, "急停后 /cmd_vel 未发布")
        max_v = max(abs(m.linear.x) for m in self.cmd_vels)
        self.assertLess(max_v, 1e-6, f"急停后线速度 {max_v} 非零")
        estop_faults = [f for f in self.faults if f.code == Fault.ESTOP and f.active]
        self.assertGreater(len(estop_faults), 0, "急停故障码未上报")

        # ---- 5. 解除急停 + RESET:系统可恢复 ----
        estop.data = False
        for _ in range(5):
            self.estop_pub.publish(estop)
            self.spin_for(0.1)
        self.spin_for(1.0)
        res = self.call_mission(MissionCommand.Request.RESET)
        self.assertTrue(res.success, f"复位失败: {res.message}")
        # 复位后可再次启动
        self.spin_for(2.0)
        self.call_mission_retry(MissionCommand.Request.START, 2)
        self.refs.clear()
        self.spin_for(5.0)
        self.assertTrue(
            any(r.tracking_active for r in self.refs), "复位后无法重新开始作业")
