#!/usr/bin/env python3
# ======================================================================
# check_viewer.py —— MuJoCo GUI 查看器可用性检测脚本（中文注释版）
# 作用：在当前终端环境中启动 MuJoCo 查看器子进程，简单走几步，
#       用于检查 WSL/桌面 OpenGL 显示环境（DISPLAY/WAYLAND/MUJOCO_GL）
#       是否配置正确、查看器能否正常打开。
# 用法：python3 scripts/check_viewer.py [--terrain stair_official] [--seconds 5.0]
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Check whether the MuJoCo GUI viewer can open in the current terminal."""

from __future__ import annotations

# 命令行参数解析
import argparse
# 操作系统接口：读取显示相关环境变量
import os
# 系统接口：把包根目录加入模块搜索路径
import sys
# 时间：控制检测时长
import time
# 路径库
from pathlib import Path


# 包根目录（scripts 的上级）
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# 把包根目录插入 sys.path，便于直接运行脚本
sys.path.insert(0, str(PACKAGE_ROOT))

# 配置类：环境配置 / 随机化配置 / 地形配置（noqa: E402 忽略导入顺序检查）
from m20_mujoco_rl.config import M20EnvConfig, RandomizationConfig, TerrainConfig  # noqa: E402
# 环境类
from m20_mujoco_rl.env import M20MujocoEnv  # noqa: E402
# 查看器子进程启动函数
from m20_mujoco_rl.viewer_process import start_viewer_process  # noqa: E402


# 解析命令行参数
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check MuJoCo viewer availability.")
    # 地形类型（默认官方台阶）
    parser.add_argument("--terrain", default="stair_official", choices=["flat", "stair_easy", "random_boxes", "stair_official"])
    # 检测时长（秒）
    parser.add_argument("--seconds", type=float, default=5.0)
    return parser.parse_args()


# 主函数
def main() -> None:
    args = parse_args()
    # 打印显示相关环境变量，方便排查 WSL/OpenGL 问题
    print(f"[INFO] DISPLAY={os.environ.get('DISPLAY')}")
    print(f"[INFO] WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}")
    print(f"[INFO] MUJOCO_GL={os.environ.get('MUJOCO_GL')}")

    # 创建环境（关闭随机化，保证检测稳定）
    env = M20MujocoEnv(
        M20EnvConfig(
            terrain=TerrainConfig(name=args.terrain),
            randomization=RandomizationConfig(enabled=False),
        )
    )
    env.reset()
    # 以实时模式启动查看器子进程
    viewer = start_viewer_process(env.xml_path, control_dt=env.cfg.control_dt, real_time=True)
    try:
        # 在检测时长内：零指令步进并同步到查看器
        end_time = time.time() + args.seconds
        while time.time() < end_time:
            env.step([0.0] * env.action_dim)
            viewer.sync(env.data.qpos.copy(), env.data.qvel.copy())
            time.sleep(env.cfg.control_dt)
    finally:
        # 关闭查看器
        viewer.close()
    # 检测结束提示
    print("[INFO] Viewer check finished. If no window appeared, fix WSL/desktop OpenGL display first.")


# 脚本入口
if __name__ == "__main__":
    main()
