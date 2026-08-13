# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Script to train RL agent with RSL-RL."""
#RSL-RL是 Robotics Systems Lab – Reinforcement Learning​ 的缩写，是 苏黎世联邦理工学院（ETH Zurich）机器人系统实验室（Robotic Systems Lab, RSL）开源的一套基于 PyTorch 的四足/双足机器人强化学习（RL）训练框架。

"""Launch Isaac Sim Simulator first."""

import argparse     # 解析命令行参数
import sys          # Python 本身和程序运行环境
import os           # 操作系统和文件系统”

from isaaclab.app import AppLauncher  # IsaacLab 应用启动器

# local imports   # 本地导入
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))) #把当前脚本的上一级目录加到 PYTHONPATH，这样下面才能直接 import cli_args
import cli_args  #通常是作者自己写的 命令行参数补充定义文件

# add argparse arguments  # 添加命令行参数定义
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.") #创建一个参数解析器
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")#命令行出现vedio就将该布尔值设置为true,否则默认为false
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
parser.add_argument(
    "--distributed", action="store_true", default=False, help="Run training with multiple GPUs or nodes."
)
# append RSL-RL cli arguments # 添加 RSL-RL 特定参数
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args # 添加 AppLauncher 特定参数
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args() # 解析参数

# always enable cameras to record video  # 如果录制视频，强制启用相机
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra 为 Hydra 配置管理器清理参数
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# suppress noisy omni.usd warnings (e.g. unresolved visual prim references)# 抑制 Omni USD 的冗余警告
import carb
carb.logging.acquire_logging().set_level_threshold_for_source(
    "omni.usd", carb.logging.LogSettingBehavior.OVERRIDE, carb.logging.LEVEL_ERROR
)

"""Check for minimum supported RSL-RL version."""

import importlib.metadata as metadata
import platform
from packaging import version

# check minimum supported rsl-rl version # 检查 RSL-RL 版本
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
    exit(1)  #在这里就是“直接干掉整个 Python 进程（整个训练脚本）”，不是只退出某个函数，也不是只退出 Isaac Lab

"""Rest everything follows."""

import gymnasium as gym # 导入 Gymnasium 环境接口
import torch # PyTorch 深度学习框架
from datetime import datetime # 获取时间戳

from rsl_rl.runners import OnPolicyRunner  # RSL-RL 在线策略训练器

from isaaclab.envs import (
    DirectMARLEnv,  # 直接多智能体环境基类
    DirectMARLEnvCfg,  # 多智能体环境配置
    DirectRLEnvCfg,  # 直接 RL 环境配置
    ManagerBasedRLEnvCfg,  # 基于管理器的 RL 环境配置
    multi_agent_to_single_agent,  # 多智能体转单智能体包装器
)
from isaaclab.utils.dict import print_dict  # 打印字典工具
from isaaclab.utils.io import dump_yaml  # YAML 文件写入工具
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
# # IsaacLab RSL-RL 相关工具，RslRlOnPolicyRunnerCfg配 PPO 超参 → RslRlVecEnvWrapper把 Isaac Lab 环境适配给 RSL-RL → handle_deprecated_rsl_rl_cfg做旧配置兼容迁移，三者共同完成 Isaac Lab ↔ RSL-RL 的对接。
from isaaclab_tasks.utils import get_checkpoint_path  # 获取检查点路径工具
from isaaclab_tasks.utils.hydra import hydra_task_config  # Hydra 任务配置装饰器

import rl_training.tasks  # noqa: F401  注册任务环境

torch.backends.cuda.matmul.allow_tf32 = True #启用TF3精度加速矩阵乘法
torch.backends.cudnn.allow_tf32 = True #启用TF32精度加速卷积
torch.backends.cudnn.deterministic = False #关闭确定性卷积（提升性能）
torch.backends.cudnn.benchmark = False #关闭cuDNN自动调优（保证一致性）


@hydra_task_config(args_cli.task, args_cli.agent) # Hydra 配置装饰器，加载任务和代理配置，task决定是哪个任务,agent决定用哪个 RL 配置（RSL‑RL PPO）
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):#env_cfg环境配置（机器人 / 观测 / 奖励 / 地形），agent_cfg：RSL‑RL PPO 配置（网络 / 超参）
    """Train with RSL-RL agent."""
    # override configurations with non-hydra CLI arguments
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    agent_cfg.max_iterations = (
        args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
    )
    #max_iterations：PPO 训练总迭代数，一次 iteration = 采样 N 步 + 一次策略更新

    # handle deprecated configurations (convert old policy format to new actor/critic format)# 处理弃用的配置（兼容旧版策略格式）
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)

    # set the environment seed# 设置环境种子
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # multi-gpu training configuration# 多 GPU 训练配置
    if args_cli.distributed:
        env_cfg.sim.device = f"cuda:{app_launcher.local_rank}"
        agent_cfg.device = f"cuda:{app_launcher.local_rank}"

        # set seed to have diversity in different threads# 为不同线程设置不同种子
        seed = agent_cfg.seed + app_launcher.local_rank
        env_cfg.seed = seed
        agent_cfg.seed = seed

    # specify directory for logging experiments # 指定实验日志目录
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Logging experiment in directory: {log_root_path}")
    # specify directory for logging runs: {time-stamp}_{run_name} # 指定运行日志目录：时间戳_运行名
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # The Ray Tune workflow extracts experiment name using the logging line below, hence, do not change it (see PR #2346, comment-2819298849)
    print(f"Exact experiment name requested from command line: {log_dir}")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root_path, log_dir)

    # create isaac environment # 创建 Isaac 仿真环境
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm# 如果 RL 算法需要，转换为单智能体实例
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # save resume path before creating a new log_dir在创建新日志目录前保存恢复路径
    if agent_cfg.resume or agent_cfg.algorithm.class_name == "Distillation":
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    # wrap for video recording为视频录制包装环境
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "train"),
            "step_trigger": lambda step: step % args_cli.video_interval == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl# 为 RSL-RL 包装环境
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # convert config to dict and create runner 将配置转为字典并创建训练器
    train_cfg = agent_cfg.to_dict()
    runner = OnPolicyRunner(env, train_cfg, log_dir=log_dir, device=agent_cfg.device)
    
    # write git state to logs# 将 Git 状态写入日志
    runner.add_git_repo_to_log(__file__)
    # load the checkpoint  加载检查点
    if agent_cfg.resume or agent_cfg.algorithm.class_name == "Distillation":
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        # load previously trained model，加载先前训练的模型
        runner.load(resume_path)

    # dump the configuration into log-directory，将配置文件存入日志目录
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    
    # run training run training  # 开始训练
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    # close the simulator 关闭仿真器
    env.close()


if __name__ == "__main__":
    # run the main function  运行主函数
    main()
    # close sim app #关闭仿真应用
    simulation_app.close()