#!/usr/bin/env python3
"""Export an M20 MuJoCo PPO checkpoint to ONNX."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export M20 policy checkpoint to ONNX.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", default="exported/m20_policy.onnx")
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


def main() -> None:
    try:
        from m20_mujoco_rl.export import export_onnx
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing export dependency. Run: "
            "python3 -m pip install -r requirements.txt"
        ) from exc

    output = export_onnx(args.checkpoint, args.output, device=args.device)
    print(f"[INFO] ONNX exported: {output}")


if __name__ == "__main__":
    args = parse_args()
    main()
