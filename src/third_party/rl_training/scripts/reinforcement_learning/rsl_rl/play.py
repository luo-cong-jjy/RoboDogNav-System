# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
from collections import deque#deque 是双端队列，相比列表 list
import math
import os
import sys

from isaaclab.app import AppLauncher

# local imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import cli_args

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
parser.add_argument("--keyboard", action="store_true", default=False, help="Whether to use keyboard.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# import after SimulationApp is created to avoid early Omniverse/pxr imports
from rl_utils import camera_follow

"""Check for minimum supported RSL-RL version."""

import importlib.metadata as metadata
import platform
from packaging import version

# check minimum supported rsl-rl version
RSL_RL_VERSION = "3.0.1"
installed_version = metadata.version("rsl-rl-lib")
if version.parse(installed_version) < version.parse(RSL_RL_VERSION):
    if platform.system() == "Windows":
        cmd = [r".\isaaclab.bat", "-p", "-m", "pip", "install", f"rsl-rl-lib=={RSL_RL_VERSION}"]
    else:
        cmd = ["./isaaclab.sh", "-p", "-m", "pip", "install", f"rsl-rl-lib=={RSL_RL_VERSION}"]
    print(
        f"Please install the correct version of RSL-RL.\nExisting version is: '{installed_version}'"
        f" and required version is: '{RSL_RL_VERSION}'.\nTo install the correct version, run:"
        f"\n\n\t{' '.join(cmd)}\n"
    )
    exit(1)

"""Rest everything follows."""

import gymnasium as gym# 导入 Gymnasium 强化学习环境标准接口
import time
import torch# 导入 PyTorch 深度学习框架

import isaaclab.utils.math as math_utils # 打印字典的工具函数

try:
    import isaacsim.util.debug_draw._debug_draw as omni_debug_draw
except Exception:
    try:
        import omni.isaac.debug_draw._debug_draw as omni_debug_draw
    except Exception:
        omni_debug_draw = None

from rsl_rl.runners import OnPolicyRunner
# 导入 RSL-RL 的在线策略运行器（PPO 算法的核心）

from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg# 导入键盘控制器
from isaaclab.envs import (
    DirectMARLEnv,  # 直接多智能体环境基类
    DirectMARLEnvCfg, # 多智能体环境配置
    DirectRLEnvCfg, # 直接 RL 环境配置
    ManagerBasedRLEnvCfg,# 基于管理器的 RL 环境配置
    multi_agent_to_single_agent,# 多智能体转单智能体包装器
)
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_jit,
    export_policy_as_onnx,
    handle_deprecated_rsl_rl_cfg,
)
from isaaclab_tasks.utils import get_checkpoint_path# 获取模型检查点路径的工具函数
from isaaclab_tasks.utils.hydra import hydra_task_config# Hydra 任务配置装饰器，自动加载 env_cfg 和 agent_cfg

# 导入自定义任务模块，触发任务注册
# noqa: F401 表示忽略“导入了但未使用”的警告，因为导入本身就会触发注册机制
import rl_training.tasks  # noqa: F401

# 装饰器：根据命令行指定的 task 和 agent，自动从配置注册表中加载 env_cfg 和 agent_cfg
@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    """Play with RSL-RL agent."""
    # 主函数：加载模型并在仿真环境中运行推理
    task_name = args_cli.task.split(":")[-1]
    # override configurations with non-hydra CLI arguments
    # 用命令行参数覆盖配置文件中的设置
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    # 如果命令行指定了环境数量，覆盖配置
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else 50

    # handle deprecated configurations (convert old policy format to new actor/critic format)
    # 处理旧版配置格式（兼容 rsl-rl 版本升级）
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    # 设置环境种子，确保测试时可复现
    env_cfg.seed = agent_cfg.seed
    # 设置仿真设备（GPU/CPU）
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # =============== 环境生成与地形配置 ===============
    # spawn the robot randomly in the grid (instead of their terrain levels)在网格中随机生成机器人（而不是它们的地形级别）
    env_cfg.scene.terrain.max_init_terrain_level = None
    # reduce the number of terrains to save memory# 减少地形网格数量以节省内存
    if env_cfg.scene.terrain.terrain_generator is not None:
        env_cfg.scene.terrain.terrain_generator.num_rows = 5
        env_cfg.scene.terrain.terrain_generator.num_cols = 5
        env_cfg.scene.terrain.terrain_generator.curriculum = False # 关闭课程学习

    # =============== 关闭训练时的随机化 (关键：测试时环境要确定) ===============       
    # disable randomization for play
    env_cfg.observations.policy.enable_corruption = False  # 关闭观测噪声
    # remove random pushing
    env_cfg.events.randomize_apply_external_force_torque = None  # 关闭外力干扰
    env_cfg.events.push_robot = None  # 关闭随机推搡
    env_cfg.curriculum.command_levels = None  # 关闭命令课程学习

    # =============== 键盘控制配置 ===============
    # 如果开启了键盘控制，强制只运行1个环境
    keyboard_command_state = None
    if args_cli.keyboard:
        env_cfg.scene.num_envs = 1
        env_cfg.terminations.time_out = None
        env_cfg.commands.base_velocity.debug_vis = False

        # 配置键盘灵敏度，基于任务配置中的范围
        config = Se2KeyboardCfg(
            v_x_sensitivity=env_cfg.commands.base_velocity.ranges.lin_vel_x[1]/2,
            v_y_sensitivity=env_cfg.commands.base_velocity.ranges.lin_vel_y[1],
            omega_z_sensitivity=env_cfg.commands.base_velocity.ranges.ang_vel_z[1],
        )
        controller = Se2Keyboard(config)

        # 定义键盘观测回调函数
        def _keyboard_obs_term(env):
            nonlocal keyboard_command_state
            keyboard_command_state = torch.tensor(controller.advance(), dtype=torch.float32).unsqueeze(0).to(env.device)
            return keyboard_command_state

        # 将键盘输入注入到观测值中
        env_cfg.observations.policy.velocity_commands = ObsTerm(
            func=_keyboard_obs_term,
        )

    # specify directory for logging experiments
    # =============== 日志与路径设置 ===============
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    # 确定模型检查点路径,# 导出模型 (JIT/ONNX 格式，用于部署)检查点”指的就是训练好的模型文件。
    # checkpoint 是载体（文件），resume 是动作
    if args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # =============== 环境创建 ===============
    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm# 如果是多智能体环境，转换为单智能体
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording# 包装视频录制器
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,# 仅在第一步触发（录制一段）
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during playback.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl# 包装 RSL-RL 向量化环境
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # =============== 模型加载与导出 ===============
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    # convert config to dict and create runner# 创建 Runner 并加载模型
    train_cfg = agent_cfg.to_dict()
    ppo_runner = OnPolicyRunner(env, train_cfg, log_dir=None, device=agent_cfg.device)
    ppo_runner.load(resume_path)

    # obtain the trained policy for inference# 获取推理策略
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")

    if version.parse(installed_version) >= version.parse("4.0.0"):
        # Use runner-native exporters for rsl-rl >= 4.0.0# RSL-RL 4.0+ 使用内置导出器
        ppo_runner.export_policy_to_jit(path=export_model_dir, filename="policy.pt")
        ppo_runner.export_policy_to_onnx(path=export_model_dir, filename="policy.onnx")
        policy_nn = None
    else:
        # Fallback for rsl-rl < 4.0.0# 旧版本兼容
        if version.parse(installed_version) >= version.parse("2.3.0"):
            policy_nn = ppo_runner.alg.policy
        else:
            policy_nn = ppo_runner.alg.actor_critic

        if hasattr(policy_nn, "actor_obs_normalizer"):
            normalizer = policy_nn.actor_obs_normalizer
        else:
            normalizer = None

        export_policy_as_onnx(
            policy=policy_nn,
            normalizer=normalizer,
            path=export_model_dir,
            filename="policy.onnx",
        )
        export_policy_as_jit(
            policy=policy_nn,
            normalizer=normalizer,
            path=export_model_dir,
            filename="policy.pt",
        )

    # =============== 足端轨迹可视化 (Debug Draw) 初始化 ===============
    # 这部分代码用于在 Isaac Sim 视窗中画出腿的运动轨迹
    dt = env.unwrapped.step_dt
    # reset environment# 重置环境
    obs, _ = env.reset()
    
    timestep = 0
    # simulate environment

    # =============== 主循环 ===============
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode# 推理模式 (不计算梯度，节省资源)
        with torch.inference_mode():
            # agent stepping# 1. Agent 决策
            actions = policy(obs)

            # env stepping# 2. 环境步进
            obs, _, _, _ = env.step(actions)

        # 3. 可视化逻辑 (Debug Draw)
        # 检查是否启用了可视化接口和必要的 ID
        if (
            VIS_ENABLEd
            and draw_interface is not None
            and foot_ids is not None
            and phase_offsets is not None
            and cycle_time is not None
            and gait_span is not None
            and gait_psi is not None
            and gait_delta is not None
            and x_offset is not None
            and stance_span is not None
            and cmd_threshold is not None
            and stand_ref_z_offset is not None
            and cmd_hist is not None
            and act_hist is not None
        ):
            local_foot_ids = foot_ids
            robot = env.unwrapped.scene["robot"]
            root_pos = robot.data.root_pos_w[0]
            root_quat = robot.data.root_quat_w[0].unsqueeze(0)

            # Initialize base-fixed stand reference once from current posture.# 初始化站立参考位置 (仅执行一次)
            if stand_ref_body is None:
                rel_init = robot.data.body_pos_w[0, local_foot_ids, :] - root_pos.unsqueeze(0)
                stand_ref_body = math_utils.quat_apply_inverse(root_quat.expand(len(local_foot_ids), -1), rel_init)
                stand_ref_body[:, 2] += stand_ref_z_offset

            # 计算经过的时间和相位
            elapsed_t = float(env.unwrapped.common_step_counter) * dt
            phase_s = torch.remainder((2.0 * elapsed_t / max(cycle_time, 1e-6)) + phase_offsets, 2.0)

            # 生成参考轨迹 (命令轨迹)
            cmd_local = _mujoco_phase_traj_body(
                phase_s=phase_s,
                gait_span=gait_span,
                gait_psi=gait_psi,
                gait_delta=gait_delta,
                x_offset=x_offset,
                stance_span=stance_span,
            )
            ref_body = stand_ref_body + cmd_local

            # 将局部坐标转换为世界坐标
            ref_world = root_pos.unsqueeze(0) + math_utils.quat_apply(root_quat.expand(len(local_foot_ids), -1), ref_body)

            actual_world = robot.data.body_pos_w[0, local_foot_ids, :]# 实际位置

            # 更新历史队列 (用于画线)
            for i in range(4):
                cmd_hist[i].append(ref_world[i].detach().cpu().tolist())
                act_hist[i].append(actual_world[i].detach().cpu().tolist())

            # 处理键盘输入的命令
            if args_cli.keyboard and keyboard_command_state is not None:
                cmd_vec = keyboard_command_state[0, :3]
            else:
                cmd_vec = env.unwrapped.command_manager.get_command("base_velocity")[0, :3]

            cmd_norm = torch.linalg.norm(cmd_vec).item()

            # 判断是否处于运动状态 (用于控制显示)
            gate_on = cmd_norm > cmd_threshold

            if args_cli.keyboard:
                ref_gate_on = cmd_norm > 0.1
            else:
                ref_gate_on = gate_on

            # 清除旧的线条
            draw_interface.clear_lines()
            starts = []
            ends = []
            colors = []
            widths = []

            # 设置透明度
            ref_alpha = 0.95 if gate_on else 0.35
            act_alpha = 0.35 if gate_on else 0.20

            # 打印 Z 轴检查信息 (调试用)
            if not phase_vis_z_printed:
                print(
                    "[INFO] phase_foot_trajectory_exp z check: "
                    f"ref_z_mean={ref_world[:, 2].mean().item():.4f}, "
                    f"act_z_mean={actual_world[:, 2].mean().item():.4f}, "
                    f"stand_ref_z_offset={stand_ref_z_offset:.4f}"
                )
                phase_vis_z_printed = True

            # 构建绘制数据
            for i in range(4):
                # 绘制实际足端轨迹 (黑色)
                act_pts = list(act_hist[i])
                for j in range(1, len(act_pts)):
                    starts.append(act_pts[j - 1])
                    ends.append(act_pts[j])
                    colors.append([0.0, 0.0, 0.0, act_alpha])
                    widths.append(1.5)

                # 绘制参考轨迹 (彩色)
                cmd_pts = list(cmd_hist[i])
                if VIS_REF_ENABLE and ref_gate_on:
                    for j in range(1, len(cmd_pts)):
                        starts.append(cmd_pts[j - 1])
                        ends.append(cmd_pts[j])
                        color = color_palette[i].copy()
                        color[3] = ref_alpha
                        colors.append(color)
                        widths.append(2.8)

            # 执行绘制
            if starts:
                draw_interface.draw_lines(starts, ends, colors, widths)

        # 视频录制结束逻辑     
        if args_cli.video:
            timestep += 1
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break
        
        # 键盘控制下的相机跟随
        if args_cli.keyboard:
            camera_follow(env)

        # 实时模式下的时间同步
        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()