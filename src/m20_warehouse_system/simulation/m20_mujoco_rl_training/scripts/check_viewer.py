#!/usr/bin/env python3
"""Check whether the MuJoCo GUI viewer can open in the current terminal."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m20_mujoco_rl.config import M20EnvConfig, RandomizationConfig, TerrainConfig  # noqa: E402
from m20_mujoco_rl.env import M20MujocoEnv  # noqa: E402
from m20_mujoco_rl.viewer_process import start_viewer_process  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check MuJoCo viewer availability.")
    parser.add_argument("--terrain", default="stair_official", choices=["flat", "stair_easy", "random_boxes", "stair_official"])
    parser.add_argument("--seconds", type=float, default=5.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print(f"[INFO] DISPLAY={os.environ.get('DISPLAY')}")
    print(f"[INFO] WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}")
    print(f"[INFO] MUJOCO_GL={os.environ.get('MUJOCO_GL')}")

    env = M20MujocoEnv(
        M20EnvConfig(
            terrain=TerrainConfig(name=args.terrain),
            randomization=RandomizationConfig(enabled=False),
        )
    )
    env.reset()
    viewer = start_viewer_process(env.xml_path, control_dt=env.cfg.control_dt, real_time=True)
    try:
        end_time = time.time() + args.seconds
        while time.time() < end_time:
            env.step([0.0] * env.action_dim)
            viewer.sync(env.data.qpos.copy(), env.data.qvel.copy())
            time.sleep(env.cfg.control_dt)
    finally:
        viewer.close()
    print("[INFO] Viewer check finished. If no window appeared, fix WSL/desktop OpenGL display first.")


if __name__ == "__main__":
    main()
