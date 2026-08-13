# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class DeeproboticsM20RoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """M20 rough 环境的 RSL-RL PPO 训练配置。

    这个文件只配置“怎么训练策略”，不配置机器人模型本身。
    机器人、观测、动作、奖励都在 env_cfg 中；这里主要是 PPO 的采样长度、
    迭代次数、网络结构、学习率、折扣因子等算法超参数。
    """

    # 每个并行环境采样 24 个 step 后，RSL-RL 会把这些 rollout 数据拿去更新一次策略。
    num_steps_per_env = 24
    # 最大 PPO 迭代次数。rough 更难，所以默认迭代次数比较大。
    max_iterations = 20000
    # 每隔 100 次迭代保存一次 checkpoint。
    save_interval = 100
    # 日志目录名的一部分：logs/rsl_rl/deeprobotics_m20_rough/...
    experiment_name = "deeprobotics_m20_rough"
    # 是否使用经验归一化。这里关闭，通常依赖 RSL-RL/环境自身的观测处理。
    empirical_normalization = False
    # 限制策略输出动作的绝对值，防止极端输出。
    clip_actions = 100
    # Actor-Critic 网络结构。
    # actor 输出动作分布，critic 估计 value；二者都用 512 -> 256 -> 128 的 MLP。
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        noise_std_type="log",
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    # PPO 算法参数。
    algorithm = RslRlPpoAlgorithmCfg(
        # value loss 权重：critic 价值函数误差在总 loss 中的占比。
        value_loss_coef=1.0,
        # 是否使用 clipped value loss，和 PPO 的“不要一次更新太猛”思想一致。
        use_clipped_value_loss=True,
        # PPO clip 参数，限制新旧策略概率比变化范围。
        clip_param=0.2,
        # 熵奖励系数，鼓励探索；太小容易早熟，太大策略可能不稳定。
        entropy_coef=0.01,
        # 每批 rollout 数据重复训练多少轮。
        num_learning_epochs=5,
        # 每轮把数据分成几个 mini-batch。
        num_mini_batches=4,
        # 学习率。
        learning_rate=1.0e-3,
        # adaptive 表示根据 KL 散度动态调整学习率。
        schedule="adaptive",
        # 折扣因子，越接近 1 越重视长期回报。
        gamma=0.99,
        # GAE(lambda) 参数，用于计算优势函数，平衡偏差和方差。
        lam=0.95,
        # 目标 KL 散度，用于 adaptive schedule 判断策略更新是否过大。
        desired_kl=0.01,
        # 梯度裁剪，避免梯度爆炸。
        max_grad_norm=1.0,
    )


@configclass
class DeeproboticsM20FlatPPORunnerCfg(DeeproboticsM20RoughPPORunnerCfg):
    """M20 flat 环境的 PPO 配置。

    平地任务更简单，所以继承 rough 配置后只缩短训练迭代次数，并修改实验名。
    """

    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 5000
        self.experiment_name = "deeprobotics_m20_flat"
