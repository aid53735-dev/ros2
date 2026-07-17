#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测量话题频率的小工具(避免 ros2 topic hz 在 WSL 下的统计口径问题)。"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, JointState


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = rclpy.create_node("rate_check")
    counts = {"imu": 0, "enc": 0, "pose": 0}
    node.create_subscription(Imu, "/imu/data",
                             lambda m: counts.__setitem__("imu", counts["imu"] + 1), 50)
    node.create_subscription(JointState, "/wheel/encoder",
                             lambda m: counts.__setitem__("enc", counts["enc"] + 1), 50)
    node.create_subscription(PoseWithCovarianceStamped, "/nav/pose",
                             lambda m: counts.__setitem__("pose", counts["pose"] + 1), 50)
    t0 = time.time()
    import threading
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()

    def run():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)

    th = threading.Thread(target=run)
    th.start()
    time.sleep(duration)
    stop.set()
    th.join()
    dt = time.time() - t0
    print(f"imu: {counts['imu']/dt:.1f} Hz, enc: {counts['enc']/dt:.1f} Hz, "
          f"pose: {counts['pose']/dt:.1f} Hz")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
