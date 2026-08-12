# 官方 rl_training 到 MuJoCo 训练包的对应关系

## 保留的思想

1. M20 是轮足机器人：
   - 12 个腿部关节走位置目标。
   - 4 个轮子关节走速度目标。

2. 策略输入保持 57 维：
   - `base_ang_vel`
   - `projected_gravity`
   - `velocity_command`
   - `joint_pos`
   - `joint_vel`
   - `last_action`

3. 动作缩放保持官方 M20 配置思路：
   - hipx: `0.125`
   - hipy/knee: `0.25`
   - wheel velocity: `5.0`

4. 训练目标沿用 rough velocity task 的核心：
   - 跟踪前进速度。
   - 保持机身姿态稳定。
   - 惩罚竖直速度、过大力矩、动作突变。
   - 在台阶/随机块地形中通过速度跟踪和稳定性奖励学会越障。

## 删除的内容

这些内容在不能使用 Isaac Sim 的情况下无法直接运行，所以没有复制：

1. Isaac Lab 场景管理：
   - `ManagerBasedRLEnv`
   - `InteractiveSceneCfg`
   - `TerrainImporterCfg`

2. Isaac Lab 传感器：
   - `RayCasterCfg`
   - `ContactSensorCfg`

3. Isaac Lab terrain generator：
   - `ROUGH_TERRAINS_CFG`
   - `pyramid_stairs`
   - `boxes`
   - `random_rough`

4. Lite3 和其他机器人配置：
   - 只保留 M20。

5. RSL-RL + Isaac Lab 的训练脚本入口：
   - 这里换成独立的轻量 PPO。

## 在 MuJoCo 中重写的内容

| 官方 rl_training / Isaac Lab | 新 MuJoCo 包 |
| --- | --- |
| `rough_env_cfg.py` | `m20_mujoco_rl/env.py` + `config.py` |
| `ActionsCfg` | `env._policy_action_to_robot_targets()` |
| `ObservationsCfg` | `env._get_obs()` |
| `RewardsCfg` | `env._compute_reward()` |
| `ROUGH_TERRAINS_CFG` | `terrain.py` |
| `train.py` | `scripts/train.py` |
| `play.py` | `scripts/play.py` |
| `export_onnx_fast.py` | `scripts/export_onnx.py` + `m20_mujoco_rl/export.py` |

## 当前还没有完全等价的部分

1. 没有 Isaac Lab 的大规模 GPU 并行环境。
   - 当前是多个 MuJoCo 环境串行采样，速度会慢很多。

2. 策略 actor 观测里没有 height scan。
   - 当前仍保持 57 维部署观测。
   - 已在奖励函数里补充 MuJoCo raycast，用来估计机身下方局部地形高度，对齐官方 `height_scanner_base` 的用途。
   - 已加入 MuJoCo 版 privileged critic：训练时 critic 可额外看到 base 线速度和 yaw 对齐高度扫描，actor/ONNX 仍保持 57 维。
   - 如果要让 actor 本身提前看到台阶，需要加入 height scan 观测；这会改变观测维度，也需要同步改 `sdk_deploy`。

3. 没有完整 domain randomization。
   - 后续应加入质量、摩擦、关节阻尼、外力扰动随机化。

4. 楼梯课程学习还很基础。
   - 当前靠 `stair_easy -> random_boxes -> stair_official` 人工分阶段训练。
   - 后续可以把台阶高度从 `0.03 m` 逐渐升到 `0.10 m`。

## 为什么当前 MuJoCo 版本不容易直接训出爬楼梯

1. 官方 `rl_training` 不是直接在 `sdk_deploy` 的 `M20_stair.xml` 里硬训。
   - 官方 rough 任务使用 Isaac Lab 的 `ROUGH_TERRAINS_CFG`，里面包含 `boxes`、`random_rough` 等大量随机地形。
   - `sdk_deploy` 里的 `M20_stair.xml` 更像部署/联调/验证场景，不是官方训练时使用的完整随机地形课程。

2. 官方训练数据量远大于当前轻量 MuJoCo 版本。
   - 官方 M20 rough PPO 默认 `max_iterations = 20000`，通常还会配合大量并行环境。
   - 当前 MuJoCo 版本是 Python 串行环境采样，即使 `--num-envs 8 --iterations 8000`，样本多样性和采样吞吐都差很多。

3. 官方 critic 可以使用更丰富的训练信息。
   - M20 配置里 actor/policy 关闭了 `height_scan`，这样更接近真机部署。
   - 但 rough 基础环境仍保留了 critic 观测组，critic 训练阶段可以获得更强的环境信息。
   - 当前 MuJoCo PPO 已改成 asymmetric actor-critic：actor 使用 57 维部署观测，critic 使用额外 base 线速度和高度扫描。

4. 原来的 MuJoCo 奖励会惩罚上楼梯。
   - 之前 `base_height` 惩罚直接使用世界系 `base_z`。
   - 机器人爬上台阶后，世界系 `base_z` 会变大，于是奖励函数误以为机身过高。
   - 现在已改为 `base_z - terrain_z`，即机身相对脚下地面的高度，对齐官方 `base_height_l2 + height_scanner_base` 的思路。

5. 当前奖励项仍比官方少。
   - 官方有 `joint_acc`、`joint_pos_limits`、`joint_power`、`stand_still`、`joint_mirror`、`undesired_contacts`、`contact_forces`、`feet_contact_without_cmd`、`upward` 等约束。
   - 当前 MuJoCo 版本只有速度跟踪、姿态、高度、力矩、动作变化和存活奖励，容易学出“乱跳但偶尔前进”的投机行为。

6. 当前还缺少完整随机化。
   - 官方会随机化摩擦、质量、质心、惯量、初始姿态、初始速度、PD 增益，还会施加外力扰动。
   - 当前 MuJoCo 版本只有很小的关节初始噪声，策略容易过拟合某一个固定状态。

7. 直接从零训练官方楼梯太难。
   - 官方楼梯台阶高度可到 `0.6 m` 的累计高度，直接从零策略开始撞这个场景，探索成功样本非常少。
   - 更合理的路线是先训平地稳定移动，再训低台阶/随机低障碍，最后逐步提高台阶高度或切换到 `stair_official` 验证。
