#!/usr/bin/env python3
# ======================================================================
# export_onnx.py —— M20 PPO 检查点 ONNX 导出脚本（中文注释版）
# 作用：把训练生成的 .pt 检查点导出为 ONNX 文件（确定性策略），
#       供 SDK 部署端推理使用。
# 用法：python3 scripts/export_onnx.py --checkpoint <policy.pt> [--output exported/m20_policy.onnx]
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Export an M20 MuJoCo PPO checkpoint to ONNX."""

from __future__ import annotations

# 命令行参数解析
import argparse
# 系统接口：把包根目录加入模块搜索路径
import sys
# 路径库
from pathlib import Path


# 包根目录（scripts 的上级）
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# 把包根目录插入 sys.path，便于直接运行脚本
sys.path.insert(0, str(PACKAGE_ROOT))

# 解析命令行参数
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export M20 policy checkpoint to ONNX.")
    # 输入检查点路径（必填）
    parser.add_argument("--checkpoint", required=True)
    # 输出 ONNX 路径（默认 exported/m20_policy.onnx）
    parser.add_argument("--output", default="exported/m20_policy.onnx")
    # 推理设备（cpu/cuda）
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


# 主函数
def main() -> None:
    try:
        # 导入导出函数（延迟导入以便给出友好报错）
        from m20_mujoco_rl.export import export_onnx
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing export dependency. Run: "
            "python3 -m pip install -r requirements.txt"
        ) from exc

    # 执行导出
    output = export_onnx(args.checkpoint, args.output, device=args.device)
    print(f"[INFO] ONNX exported: {output}")


# 脚本入口
if __name__ == "__main__":
    args = parse_args()
    main()
