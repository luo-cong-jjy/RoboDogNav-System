"""Small quaternion helpers using MuJoCo's wxyz convention."""

from __future__ import annotations

import numpy as np


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    out = np.array(q, dtype=np.float64, copy=True)
    out[1:] *= -1.0
    return out


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quat_rotate_inverse(q: np.ndarray, vec: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    q = q / max(np.linalg.norm(q), 1.0e-8)
    vq = np.array([0.0, vec[0], vec[1], vec[2]], dtype=np.float64)
    return quat_mul(quat_mul(quat_conjugate(q), vq), q)[1:]


def wrap_to_pi(values: np.ndarray) -> np.ndarray:
    return (values + np.pi) % (2.0 * np.pi) - np.pi

