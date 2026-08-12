"""M20 constants shared by MuJoCo training, ONNX export and SDK deployment."""

from __future__ import annotations

import numpy as np


ROBOT_ORDER = [
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
]

POLICY_ORDER = [
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
    "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
]

ROBOT_TO_POLICY = np.array([ROBOT_ORDER.index(name) for name in POLICY_ORDER], dtype=np.int64)
POLICY_TO_ROBOT = np.array([POLICY_ORDER.index(name) for name in ROBOT_ORDER], dtype=np.int64)

LEG_POLICY_INDICES = np.arange(12, dtype=np.int64)
WHEEL_POLICY_INDICES = np.arange(12, 16, dtype=np.int64)
WHEEL_ROBOT_INDICES = np.array([3, 7, 11, 15], dtype=np.int64)

DEFAULT_POLICY_POS = np.array(
    [
        0.0, -0.3, 0.6,
        0.0, -0.3, 0.6,
        0.0, 0.3, -0.6,
        0.0, 0.3, -0.6,
        0.0, 0.0, 0.0, 0.0,
    ],
    dtype=np.float64,
)
DEFAULT_ROBOT_POS = DEFAULT_POLICY_POS[POLICY_TO_ROBOT]

ACTION_SCALE_POLICY = np.array(
    [
        0.125, 0.25, 0.25,
        0.125, 0.25, 0.25,
        0.125, 0.25, 0.25,
        0.125, 0.25, 0.25,
        5.0, 5.0, 5.0, 5.0,
    ],
    dtype=np.float64,
)

KP_ROBOT = np.array([80.0, 80.0, 80.0, 0.0] * 4, dtype=np.float64)
KD_ROBOT = np.array([2.0, 2.0, 2.0, 0.6] * 4, dtype=np.float64)

OBS_DIM = 57
ACTION_DIM = 16
GRAVITY_VEC = np.array([0.0, 0.0, -1.0], dtype=np.float64)

