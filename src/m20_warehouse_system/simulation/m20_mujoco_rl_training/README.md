# M20 MuJoCo RL Training

这个包是给 M20 轮足机器人准备的轻量强化学习训练包。它参考了官方
`src/third_party/rl_training` 中 M20 rough velocity task 的设计，但不依赖
Isaac Sim / Isaac Lab，训练后端改为 MuJoCo。

## 为什么不是直接改官方 rl_training

官方 `rl_training` 的核心依赖是 Isaac Lab：

- 场景、地形、传感器、并行环境来自 Isaac Lab。
- PPO 训练流程依赖 Isaac Lab 环境接口。
- M20 任务配置里的 `ROUGH_TERRAINS_CFG`、`RayCasterCfg`、`ContactSensorCfg`
  都不是 MuJoCo 对象。

所以如果不能使用 Isaac Sim，最稳妥的路线不是“删几个文件继续跑”，而是保留
M20 的观测、动作、奖励思想，重新写一个 MuJoCo 环境。

## 当前保留的 M20 训练约定

1. 动作维度：16
   - 12 个腿部关节位置目标。
   - 4 个轮子关节速度目标。
2. 观测维度：57
   - 3 维机体角速度。
   - 3 维 projected gravity。
   - 3 维速度命令。
   - 16 维关节相对位置，轮子位置置零。
   - 16 维关节速度。
   - 16 维上一帧动作。
3. 执行器：PD 控制
   - 腿部：位置 PD。
   - 轮子：速度阻尼。
4. ONNX 导出接口：
   - input name: `obs`
   - output name: `actions`

这些约定和 `sdk_deploy/src/M20_sdk_deploy/run_policy/m20_policy_runner.hpp`
保持一致，后续可以把训练得到的 ONNX 替换到 `sdk_deploy`。

## 目录结构

```text
m20_mujoco_rl_training/
├── m20_mujoco_rl/
│   ├── config.py          # 环境和 PPO 配置
│   ├── constants.py       # M20 关节顺序、默认姿态、PD 参数
│   ├── env.py             # MuJoCo 强化学习环境
│   ├── export.py          # checkpoint -> ONNX
│   ├── ppo.py             # 轻量 PPO 实现
│   └── terrain.py         # 平地/台阶/随机块地形 XML 生成
├── scripts/
│   ├── train.py           # 训练入口
│   ├── evaluate.py        # 无窗口固定评估
│   ├── play.py            # 回放 checkpoint
│   └── export_onnx.py     # 导出 ONNX
├── requirements.txt
└── pyproject.toml
```

## 安装依赖

当前工作区已经有 `mujoco` 和 `numpy`，但这个环境里暂时没有 `torch`。训练前需要安装：

```bash
cd /home/virdyn/robodog_nav_system/src/third_party/m20_mujoco_rl_training
python3 -m pip install -r requirements.txt
```

`torch` 体积较大，CPU/GPU 版本请按你的机器条件安装。这个包本身不需要
`gymnasium`、`rsl_rl`、`isaaclab`。

## 推荐训练流程

默认训练环境使用包内复制的官方 MuJoCo 楼梯场景：

```text
assets/m20_mjcf/mjcf/M20_stair.xml
```

如果需要临时切换到别的 M20 MJCF 目录，可以设置环境变量
`M20_MJCF_DIR`，或者直接用训练/评估脚本的 `--model-xml` 指定 XML。

推荐先用课程训练入口，它会按顺序训练：

```text
flat -> stair_easy -> random_boxes -> stair_official
```

每个阶段会自动从上一阶段 checkpoint 继续。现在训练器会额外做固定
deterministic 评估；如果上一阶段存在
`policy_eval_best.pt`，课程训练会优先从它继续，否则才使用 `policy_latest.pt`。

```bash
python3 scripts/train_curriculum.py \
  --num-envs 8 \
  --steps-per-env 24 \
  --run-name m20_curriculum_stable
```

训练默认启用 MuJoCo 版 domain/reset randomization，包括：

- reset 初始 base 姿态、速度和关节状态扰动。
- base 质量、非 base link 质量、base COM、接触摩擦随机化。
- 腿/轮 PD 增益缩放。
- reset 短时外力/力矩扰动和 episode 中的随机 push。

如果需要做固定物理参数的对照实验，可以加：

```bash
python3 scripts/train_curriculum.py \
  --num-envs 8 \
  --run-name m20_curriculum_no_randomization \
  --no-randomization
```

默认课程迭代数是：

```text
flat: 1000
stair_easy: 3000
random_boxes: 5000
stair_official: 8000
```

可以用 `--flat-iterations`、`--stair-easy-iterations`、
`--random-boxes-iterations`、`--stair-official-iterations` 单独覆盖。把某个阶段
设为 `0` 就会跳过该阶段。

如果 viewer 里出现启动时腿部姿态异常、动作打满或到台阶前抬腿摔倒，先不要从旧
checkpoint 继续训。推荐用更保守的稳定版课程从头重训，先跳过 `random_boxes`：

```bash
python3 scripts/train_curriculum.py \
  --num-envs 8 \
  --steps-per-env 24 \
  --flat-iterations 1500 \
  --stair-easy-iterations 3000 \
  --random-boxes-iterations 0 \
  --stair-official-iterations 3000 \
  --stair-easy-height 0.03 \
  --run-name m20_stable_v2_phase1
```

通过 `stair_official` 固定物理参数评估后，再逐步加回 `random_boxes` 和更大的
随机化强度。

如果回放时机器人乱跳，先看训练日志里的 `entropy`：

- 正常保守训练初期通常在个位数以内。
- 如果冲到几十、上百，比如 `entropy=227`，说明策略探索标准差已经爆炸。
- 这种 checkpoint 基本不能用，建议丢弃并用当前稳定版重新训练。

当前训练器已经对动作和 `log_std` 做了限制。默认策略动作上限是 `0.60`，
并且 PPO 更新会直接约束 deterministic actor mean，避免导出后的策略长期贴着
动作边界。需要单独训练某一阶段时，也可以使用 `scripts/train.py`：

```bash
python3 scripts/train.py \
  --terrain stair_official \
  --num-envs 8 \
  --iterations 8000 \
  --run-name official_stair_stable
```

现在更推荐使用官方风格 preset，把常用训练参数收进配置里。当前低台阶续训可以简化成：

```bash
python3 scripts/train.py \
  --preset official_like_stair \
  --resume-checkpoint ../third_party/m20_mujoco_rl_training/logs/m20_mujoco/m20_stair_easy_no_scrape_v1/policy_eval_best.pt \
  --run-name m20_stair_easy_0023_privcritic_v1
```

`official_like_stair` 默认会启用特权 critic 高度扫描、低台阶高度课程、接触终止、较强探索和固定评估。需要微调时只写覆盖项，例如：

```bash
python3 scripts/train.py \
  --preset official_like_stair \
  --iterations 4000 \
  --stair-height-range 0.020 0.026 \
  --resume-checkpoint logs/m20_mujoco/<run_name>/policy_eval_best.pt \
  --run-name m20_stair_easy_0026_privcritic_v1
```

如果 2.3cm 能过但动作长期打满，先用 soft refine 稳住低台阶能力并压低动作幅值：

```bash
python3 scripts/train.py \
  --preset official_like_stair_soft_refine \
  --resume-checkpoint logs/m20_mujoco/m20_stair_easy_0023_privcritic_v1/policy_eval_best.pt \
  --run-name m20_stair_easy_0023_soft_refine_v1
```

往官方 rough terrain 思路靠时，可以先训随机低障碍阶段：

```bash
python3 scripts/train.py \
  --preset official_like_rough \
  --resume-checkpoint logs/m20_mujoco/<run_name>/policy_eval_best.pt \
  --run-name m20_random_boxes_privcritic_v1
```

单阶段续训或手动接力可以用：

```bash
python3 scripts/train.py \
  --terrain stair_official \
  --resume-checkpoint logs/m20_mujoco_curriculum/<run_name>/03_random_boxes/policy_latest.pt \
  --run-name official_stair_resume
```

如果是在已有稳定 checkpoint 上做“降低动作幅值”的短续训，优先从
`policy_eval_best.pt` 开始，并观察日志里的 `mean_action`、`mean_max`、
`eval.action_abs_mean` 和 `eval.action_saturation_mean`：

```bash
python3 scripts/train.py \
  --terrain stair_official \
  --resume-checkpoint logs/m20_mujoco/<run_name>/policy_eval_best.pt \
  --iterations 800 \
  --num-envs 8 \
  --steps-per-env 24 \
  --run-name m20_stair_soft_action_refine
```

如果策略能平地跑但碰不到/爬不上官方楼梯，先做低台阶无刮碰课程。官方 0.1m 台阶
太容易诱导策略用机身/腿部硬蹭，第一阶段先用 `stair_easy` 的 0.02m 台阶，并开启
非轮接触终止，让策略只能用轮子和合理姿态上去：

```bash
python3 scripts/train.py \
  --terrain stair_easy \
  --stair-height 0.02 \
  --stair-count 3 \
  --resume-checkpoint logs/m20_mujoco/m20_stair_soft_action_v2/policy_eval_best.pt \
  --iterations 2000 \
  --num-envs 8 \
  --steps-per-env 24 \
  --episode-seconds 8 \
  --base-init-x 0.85 \
  --base-x-range -0.10 0.10 \
  --forward-command-range 0.35 0.65 \
  --progress-weight 1.0 \
  --terrain-height-progress-weight 10.0 \
  --stair-height-weight 4.0 \
  --undesired-contact-weight -20.0 \
  --contact-force-weight -0.0003 \
  --terminate-on-undesired-contact \
  --undesired-contact-terminal-penalty -500 \
  --action-limit 0.75 \
  --mean-action-saturation-threshold 0.65 \
  --eval-interval 50 \
  --eval-episodes 3 \
  --run-name m20_stair_easy_no_scrape_v1
```

训练中看 `eval max_x`、`eval max_h`、`undesired` 和 `contact`：`max_h > 0`
表示上到台阶地形，`undesired=0` 才说明不是靠机身/腿部硬蹭。

训练日志和 checkpoint 默认放在：

```text
logs/m20_mujoco/<run_name>/
logs/m20_mujoco_curriculum/<run_name>/<stage_name>/
```

每个 run 会写出：

- `config.json`：本次环境、奖励和 PPO 配置。
- `metrics.jsonl`：每轮训练指标，包括总 reward、关键 reward terms、动作幅值和
  `log_std`。
- `eval_metrics.jsonl`：固定物理参数 deterministic 评估指标。
- `policy_latest.pt`：最近一次 checkpoint。
- `policy_best.pt`：产生 episode return 后，目前 episode return 最好的 checkpoint。
- `policy_eval_best.pt`：固定评估分数最好的 checkpoint，部署和导出优先看这个。

课程训练还会在根目录写出 `curriculum.json`，记录每个阶段的地形、日志目录、
输入 checkpoint 和输出 checkpoint。

回放：

```bash
python3 scripts/play.py \
  --checkpoint logs/m20_mujoco/<run_name>/policy_latest.pt \
  --terrain stair_official \
  --seconds 30
```

`play.py --seconds` 现在也会默认作为 episode 长度，避免 8 秒自动 reset 导致还没到
楼梯就重新开始。可以用 `--base-init-x 1.0` 从楼梯前直接观察接触段。

训练时打开 MuJoCo 实时窗口：

```bash
python3 scripts/train.py \
  --terrain stair_official \
  --num-envs 1 \
  --iterations 1000 \
  --render \
  --render-real-time
```

说明：

- `--render` 只显示其中一个训练环境，默认是第 0 个。
- `--render-env 3` 可以改成显示第 3 个环境。
- `--render-real-time` 会尽量按真实时间播放，但会明显降低训练速度。
- 正式长时间训练建议不要开 viewer；先训练，再用 `scripts/play.py` 回放效果。

如果 `--render` 没有窗口，先单独检查 MuJoCo GUI：

```bash
python3 scripts/check_viewer.py --terrain stair_official --seconds 5
```

如果终端出现 `X11: Failed to open display` 或 `could not initialize GLFW`，
说明当前 WSL/桌面图形显示没有打通。这时训练可以继续跑，但 MuJoCo 窗口打不开；
需要先修 WSLg/X11/OpenGL 环境，或者先不开 `--render`，训练后再在可显示的终端中回放。

导出 ONNX：

```bash
python3 scripts/export_onnx.py \
  --checkpoint logs/m20_mujoco/<run_name>/policy_latest.pt \
  --output exported/m20_policy.onnx
```

替换部署权重时，备份原权重后再复制：

```bash
cp /home/virdyn/robodog_nav_system/src/third_party/sdk_deploy/src/M20_sdk_deploy/policy/policy.onnx \
   /home/virdyn/robodog_nav_system/src/third_party/sdk_deploy/src/M20_sdk_deploy/policy/policy.original.onnx

cp exported/m20_policy.onnx \
   /home/virdyn/robodog_nav_system/src/third_party/sdk_deploy/src/M20_sdk_deploy/policy/policy.onnx
```

## 重要提醒

第一版目标是跑通“MuJoCo 可训练闭环”。爬楼梯想训到稳定，后面还需要继续加：

- 更强的地形课程学习。
- 更细的接触奖励。
- 更合理的摔倒终止条件。
- domain randomization。
- 可能还需要 height scan，否则策略无法提前看见台阶，只能靠身体反馈反应。
