# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class DeeproboticsLite3RoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """Lite3 rough 环境的 RSL-RL PPO 训练配置。

    这个类只描述“算法怎么训练”，不描述机器人模型和奖励。
    机器人/观测/动作/奖励在 ``rough_env_cfg.py`` 中配置。
    """

    # 每个并行环境连续采样多少个控制步，然后把这些 rollout 数据拿去做一次 PPO 更新。
    num_steps_per_env = 24
    # 最大 PPO 迭代次数；rough 任务比 flat 难，通常需要更多迭代。
    max_iterations = 10000
    # 每隔多少次迭代保存一次 checkpoint。
    save_interval = 100
    # 日志目录名，最终路径通常是 logs/rsl_rl/deeprobotics_lite3_rough/时间戳。
    experiment_name = "deeprobotics_lite3_rough"
    # 是否使用经验归一化；这里关闭，依赖环境内的 scale/noise/clip。
    empirical_normalization = False
    # 限制动作输出范围，防止策略初期输出极端值。
    clip_actions = 100
    # Actor-Critic 网络结构。
    # actor 输出动作分布，critic 估计状态价值 V(s)。
    policy = RslRlPpoActorCriticCfg(
        # 初始动作探索噪声标准差，越大探索越激进。
        init_noise_std=1.0,
        # 使用 log_std 参数化标准差，训练时数值更稳定。
        noise_std_type="log",
        # actor MLP 隐藏层宽度。
        actor_hidden_dims=[512, 256, 128],
        # critic MLP 隐藏层宽度。
        critic_hidden_dims=[512, 256, 128],
        # ELU 是机器人 RL 中常见激活函数，负半轴比 ReLU 更平滑。
        activation="elu",
    )
    # PPO 核心超参数。
    algorithm = RslRlPpoAlgorithmCfg(
        # value loss 在总损失中的权重。
        value_loss_coef=1.0,
        # 使用 clipped value loss，限制 critic 更新幅度。
        use_clipped_value_loss=True,
        # PPO clip 系数，限制新旧策略概率比变化。
        clip_param=0.2,
        # 熵奖励系数，鼓励探索，防止策略过早收敛。
        entropy_coef=0.01,
        # 同一批 rollout 数据重复训练几轮。
        num_learning_epochs=5,
        # 每轮把 rollout 数据切成几个 mini-batch。
        num_mini_batches=4,
        # Adam 学习率。
        learning_rate=1.0e-3,
        # adaptive 会根据 KL 散度自动调节学习率。
        schedule="adaptive",
        # 折扣因子，越接近 1 越看重长期回报。
        gamma=0.99,
        # GAE lambda，平衡优势估计的偏差和方差。
        lam=0.95,
        # 目标 KL；超过太多说明策略更新过猛。
        desired_kl=0.01,
        # 梯度裁剪阈值，防止梯度爆炸。
        max_grad_norm=1.0,
    )


@configclass
class DeeproboticsLite3FlatPPORunnerCfg(DeeproboticsLite3RoughPPORunnerCfg):
    """Lite3 flat 环境的 PPO 配置。

    继承 rough 的网络结构和算法参数，只修改训练轮数和实验名。
    """

    def __post_init__(self):
        # 先继承 rough 配置。
        super().__post_init__()

        # 平地任务相对简单，但这里仍保留 10000 次迭代作为默认值。
        self.max_iterations = 10000
        # 修改日志目录名，避免和 rough 实验混在一起。
        self.experiment_name = "deeprobotics_lite3_flat"
