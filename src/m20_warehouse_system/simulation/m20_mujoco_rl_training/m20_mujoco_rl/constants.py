# ======================================================================
# constants.py —— M20 训练/导出/部署共享常量（中文注释版）
# 作用：定义机器人关节顺序（机器人系/策略系）、动作缩放、PD 增益、
#       观测/动作维度等，供 MuJoCo 训练、ONNX 导出与 SDK 部署共用，
#       保证三者的关节与动作接口完全一致
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""M20 constants shared by MuJoCo training, ONNX export and SDK deployment."""

from __future__ import annotations

# NumPy：数组定义
import numpy as np


# 机器人（SDK/模型）关节顺序：fl/fr/hl/hr = 左前/右前/左后/右后，
# 每条腿依次为 hipx / hipy / knee / wheel 四个关节，共 16 个
ROBOT_ORDER = [
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
]

# 策略（训练）关节顺序：先排 12 个腿部关节（hipx/hipy/knee x4 腿），
# 再排 4 个轮子关节（fl/fr/hl/hr wheel）
POLICY_ORDER = [
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
    "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
]

# 机器人序 -> 策略序的映射索引（用于把机器人关节数据重排为策略序）
ROBOT_TO_POLICY = np.array([ROBOT_ORDER.index(name) for name in POLICY_ORDER], dtype=np.int64)
# 策略序 -> 机器人序的映射索引（用于把策略输出重排为机器人关节顺序）
POLICY_TO_ROBOT = np.array([POLICY_ORDER.index(name) for name in ROBOT_ORDER], dtype=np.int64)

# 策略序中的腿部关节索引（0..11）
LEG_POLICY_INDICES = np.arange(12, dtype=np.int64)
# 策略序中的轮子关节索引（12..15）
WHEEL_POLICY_INDICES = np.arange(12, 16, dtype=np.int64)
# 机器人序中的轮子关节索引（每腿第 4 个：3/7/11/15）
WHEEL_ROBOT_INDICES = np.array([3, 7, 11, 15], dtype=np.int64)

# 策略序下的默认关节位置（弧度）：四条腿的 hipx/hipy/knee 默认姿态 + 轮子为 0
DEFAULT_POLICY_POS = np.array(
    [
        0.0, -0.3, 0.6,   # fl（左前）
        0.0, -0.3, 0.6,   # fr（右前）
        0.0, 0.3, -0.6,   # hl（左后）
        0.0, 0.3, -0.6,   # hr（右后）
        0.0, 0.0, 0.0, 0.0,  # 四个轮子
    ],
    dtype=np.float64,
)
# 机器人序下的默认关节位置（由策略序经重排映射得到）
DEFAULT_ROBOT_POS = DEFAULT_POLICY_POS[POLICY_TO_ROBOT]

# 策略动作缩放系数（策略输出是归一化指令）：
# 腿部关节乘 0.125/0.25 弧度，轮子乘 5 rad/s
ACTION_SCALE_POLICY = np.array(
    [
        0.125, 0.25, 0.25,   # fl 腿
        0.125, 0.25, 0.25,   # fr 腿
        0.125, 0.25, 0.25,   # hl 腿
        0.125, 0.25, 0.25,   # hr 腿
        5.0, 5.0, 5.0, 5.0,  # 四个轮子（角速度）
    ],
    dtype=np.float64,
)

# 机器人序下各关节的 PD 位置增益（kp）：腿部 80，轮子 0（轮子只控速度）
KP_ROBOT = np.array([80.0, 80.0, 80.0, 0.0] * 4, dtype=np.float64)
# 机器人序下各关节的 PD 速度增益（kd）：腿部 2.0，轮子 0.6
KD_ROBOT = np.array([2.0, 2.0, 2.0, 0.6] * 4, dtype=np.float64)

# 策略观测维度（57 维）
OBS_DIM = 57
# 动作维度（16 个关节）
ACTION_DIM = 16
# 重力方向向量（机体系中表示，用于投影重力观测）
GRAVITY_VEC = np.array([0.0, 0.0, -1.0], dtype=np.float64)
