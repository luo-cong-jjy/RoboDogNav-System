"""Verify that the flat ROS 2 waypoint array is accepted at startup."""
# ============================================================
# 文件职责：launch 测试——验证 scan_planner_node 在启动时能够接受
#   "扁平化"的 ROS 2 路点参数数组（fsm.waypoints，由路点 YAML 提供）。
#   通过 launch_testing 拉起规划器节点，并断言进程在超时内成功启动。
# ============================================================

import os

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    # 函数：生成 launch 测试描述（launch_testing 固定接口）
    # 返回：(LaunchDescription, fixtures 字典)；fixtures 供测试用例注入使用
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    # 计算包根目录（test/ 的上一级目录）
    planner = launch_ros.actions.Node(
        package="m20_scan_planner",        # 目标 ROS 2 包名
        executable="scan_planner_node",    # 待测试的可执行文件
        name="scan_planner_node",
        parameters=[
            os.path.join(root, "config", "planner.yaml"),           # 规划器主配置文件
            os.path.join(root, "config", "keypoints.example.yaml"), # 含路点数组的示例配置
        ],
        output="screen", # 节点输出打到屏幕
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        # LaunchDescription：启动规划器 + ReadyToTest 动作（就绪后才执行测试用例）
        {"planner": planner},
    )


class TestWaypointParameters:
    def test_process_starts(self, proc_info, planner):
        # 测试用例：规划器进程应在 15 秒内成功启动（否则启动参数有问题）
        proc_info.assertWaitForStartup(process=planner, timeout=15)
