# -*- coding: utf-8 -*-
"""bringup.launch.py:一键拉起全部导航节点(可选 use_sim 拉起仿真)。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tower_nav")
    sim_share = get_package_share_directory("tower_nav_sim")

    config = os.path.join(pkg_share, "config")
    use_sim = LaunchConfiguration("use_sim")
    traj_dir_arg = LaunchConfiguration("trajectory_dir")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim", default_value="true", description="是否拉起仿真节点"),
        DeclareLaunchArgument(
            "trajectory_dir",
            default_value=os.path.join(config, "trajectories"),
            description="轨迹文件目录"),

        # 仿真(可选)
        Node(
            package="tower_nav_sim",
            executable="sim_node",
            name="tower_nav_sim",
            parameters=[os.path.join(sim_share, "config", "sim.yaml")],
            condition=IfCondition(use_sim),
            output="screen",
        ),

        # 定位融合
        Node(
            package="tower_nav",
            executable="localization_node",
            name="localization_node",
            parameters=[os.path.join(config, "localization.yaml")],
            output="screen",
        ),

        # 轨迹跟踪
        Node(
            package="tower_nav",
            executable="trajectory_node",
            name="trajectory_node",
            parameters=[
                os.path.join(config, "mission.yaml"),
                {"trajectory_dir": traj_dir_arg},
            ],
            output="screen",
        ),

        # 导航控制器
        Node(
            package="tower_nav",
            executable="controller_node",
            name="controller_node",
            parameters=[os.path.join(config, "control.yaml")],
            output="screen",
        ),

        # 异常管理
        Node(
            package="tower_nav",
            executable="exception_node",
            name="exception_node",
            parameters=[os.path.join(config, "mission.yaml")],
            output="screen",
        ),

        # 任务管理
        Node(
            package="tower_nav",
            executable="mission_node",
            name="mission_node",
            output="screen",
        ),

        # 状态管理
        Node(
            package="tower_nav",
            executable="status_node",
            name="status_node",
            output="screen",
        ),
    ])
