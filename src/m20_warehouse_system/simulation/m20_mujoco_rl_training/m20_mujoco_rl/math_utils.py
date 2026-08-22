# ======================================================================
# math_utils.py —— 四元数小工具函数（中文注释版）
# 作用：提供基于 MuJoCo wxyz（w 在前）约定的四元数共轭/乘法/旋转向量运算，
#       以及角度归一化工具，供环境观测与奖励计算使用
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Small quaternion helpers using MuJoCo's wxyz convention."""

from __future__ import annotations

# NumPy：数组运算
import numpy as np


# 四元数共轭（虚部取反；单位四元数的共轭即逆）
def quat_conjugate(q: np.ndarray) -> np.ndarray:
    # 复制输入（wxyz 顺序）
    out = np.array(q, dtype=np.float64, copy=True)
    # 虚部 x/y/z 取反
    out[1:] *= -1.0
    return out


# 四元数乘法 q1 * q2（wxyz 顺序，返回归一化前的结果）
def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    # 解包两个四元数
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    # 标准四元数乘法公式
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,   # w
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,   # x
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,   # y
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,   # z
        ],
        dtype=np.float64,
    )


# 用四元数 q 的逆旋转向量 vec（即把向量从世界系转到机体系）
def quat_rotate_inverse(q: np.ndarray, vec: np.ndarray) -> np.ndarray:
    # 归一化四元数
    q = np.asarray(q, dtype=np.float64)
    q = q / max(np.linalg.norm(q), 1.0e-8)
    # 把向量补齐为纯四元数（w=0）
    vq = np.array([0.0, vec[0], vec[1], vec[2]], dtype=np.float64)
    # 旋转公式：q* ⊗ v ⊗ q，返回虚部（向量部分）
    return quat_mul(quat_mul(quat_conjugate(q), vq), q)[1:]


# 把角度数组归一化到 [-pi, pi]
def wrap_to_pi(values: np.ndarray) -> np.ndarray:
    # 取模后平移：先加 pi 取 2pi 模，再减 pi
    return (values + np.pi) % (2.0 * np.pi) - np.pi
