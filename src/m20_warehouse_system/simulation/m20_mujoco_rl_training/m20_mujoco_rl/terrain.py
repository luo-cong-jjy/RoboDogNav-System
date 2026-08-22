# ======================================================================
# terrain.py —— M20 MuJoCo 训练地形生成（中文注释版）
# 作用：根据 TerrainConfig 选择/生成地形 XML：
#   - 指定 model_xml 时直接用该模型
#   - stair_official：使用官方 M20_stair.xml（含 scene.xml 网格场景）
#   - flat / stair_easy / random_boxes：基于 M20.xml 程序化生成地形
#     （在 worldbody 中追加台阶或随机箱体），写入临时目录并缓存
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""MJCF terrain generation for M20 MuJoCo training."""

from __future__ import annotations

# 哈希：生成缓存文件名
import hashlib
# JSON：序列化地形配置
import json
# 临时目录
import tempfile
# 数据类转字典
from dataclasses import asdict
# XML 解析：读取/修改 M20.xml
import xml.etree.ElementTree as ET
# 路径库
from pathlib import Path

# NumPy：随机箱体生成
import numpy as np

# 配置类：地形配置 / 默认 MJCF 目录
from .config import TerrainConfig, default_m20_mjcf_dir


# 基础 M20 模型路径（仅机器人本体，无地形）
def _base_m20_xml() -> Path:
    return default_m20_mjcf_dir() / "M20.xml"


# 官方带台阶的模型路径（含 scene.xml 场景网格）
def _official_stair_xml() -> Path:
    return default_m20_mjcf_dir() / "M20_stair.xml"


# 解析最终要加载的模型 XML 路径
def resolve_model_xml(model_xml: str | None, terrain: TerrainConfig) -> Path:
    # 显式指定模型：直接使用
    if model_xml is not None:
        return Path(model_xml).expanduser().resolve()
    # 官方台阶地形：使用 M20_stair.xml
    if terrain.name == "stair_official":
        return _official_stair_xml().resolve()
    # 其它地形：程序化生成
    return build_generated_xml(terrain)


# 基于 M20.xml 生成带地形的新 XML（flat 不做修改，stair_easy 加台阶，random_boxes 加随机箱体）
def build_generated_xml(terrain: TerrainConfig) -> Path:
    # 读取基础 M20.xml
    base_xml = _base_m20_xml().resolve()
    tree = ET.parse(base_xml)
    root = tree.getroot()

    # 修正 compiler 的 meshdir，指向实际 meshes 目录（相对路径依赖原目录结构）
    mesh_dir = base_xml.parents[1] / "meshes"
    for compiler in root.findall("compiler"):
        if "meshdir" in compiler.attrib:
            compiler.set("meshdir", str(mesh_dir.resolve()))

    # 找到 worldbody 节点，用于追加地形
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError(f"Cannot find <worldbody> in {base_xml}")

    # 按地形类型生成：
    if terrain.name == "flat":
        # 平地：不追加任何地形
        pass
    elif terrain.name == "stair_easy":
        # 简易台阶
        _add_stairs(worldbody, terrain)
    elif terrain.name == "random_boxes":
        # 随机箱体
        _add_random_boxes(worldbody, terrain)
    else:
        raise ValueError(
            f"Unsupported generated terrain: {terrain.name}. "
            "Use flat, stair_easy, random_boxes or stair_official."
        )

    # 输出到系统临时目录下的 m20_mujoco_rl_training 文件夹
    out_dir = Path(tempfile.gettempdir()) / "m20_mujoco_rl_training"
    out_dir.mkdir(parents=True, exist_ok=True)
    # 用配置的序列化哈希作为文件名一部分，相同配置直接复用缓存
    terrain_key = json.dumps(asdict(terrain), sort_keys=True, separators=(",", ":"))
    terrain_hash = hashlib.sha1(terrain_key.encode("utf-8")).hexdigest()[:10]
    out_path = out_dir / f"M20_{terrain.name}_{terrain.seed}_{terrain_hash}.xml"
    # 写回 XML
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path


# 在 worldbody 中追加一组台阶 + 末端平台
def _add_stairs(worldbody: ET.Element, terrain: TerrainConfig) -> None:
    # 创建台阶父 body
    stair_body = ET.SubElement(worldbody, "body", {"name": "training_staircase", "pos": "0 0 0"})
    # 逐级生成台阶箱体
    for i in range(terrain.stair_count):
        # 第 i+1 级顶部高度
        top = terrain.stair_height * (i + 1)
        # 该级台阶中心 x
        center_x = terrain.stair_start_x + i * terrain.stair_depth + terrain.stair_depth / 2.0
        # 箱体几何属性：box 类型，位置/尺寸按台阶参数计算
        geom = {
            "name": f"train_stair_{i + 1}",
            "type": "box",
            "pos": f"{center_x:.4f} 0 {top / 2.0:.4f}",
            "size": f"{terrain.stair_depth / 2.0:.4f} 2.0 {top / 2.0:.4f}",
            "rgba": "0.45 0.45 0.45 1",
            "condim": "3",
            "friction": "1 0.01 0.01",
        }
        ET.SubElement(stair_body, "geom", geom)

    # 台阶走完后加一段等高平台
    final_top = terrain.stair_height * terrain.stair_count
    plateau_x = terrain.stair_start_x + terrain.stair_count * terrain.stair_depth + terrain.plateau_length / 2.0
    ET.SubElement(
        stair_body,
        "geom",
        {
            "name": "train_stair_plateau",
            "type": "box",
            "pos": f"{plateau_x:.4f} 0 {final_top / 2.0:.4f}",
            "size": f"{terrain.plateau_length / 2.0:.4f} 2.0 {final_top / 2.0:.4f}",
            "rgba": "0.45 0.45 0.45 1",
            "condim": "3",
            "friction": "1 0.01 0.01",
        },
    )


# 在 worldbody 中追加随机放置的箱体障碍物
def _add_random_boxes(worldbody: ET.Element, terrain: TerrainConfig) -> None:
    # 用地形种子初始化随机数生成器（保证可复现）
    rng = np.random.default_rng(terrain.seed)
    # 创建箱体父 body
    body = ET.SubElement(worldbody, "body", {"name": "training_random_boxes", "pos": "0 0 0"})
    # 生成指定数量的随机箱体
    for i in range(terrain.random_box_count):
        # 随机箱体高度/尺寸/位置
        height = rng.uniform(*terrain.random_box_height)
        size_x = rng.uniform(0.08, 0.25)
        size_y = rng.uniform(0.08, 0.25)
        x = rng.uniform(*terrain.random_box_area_x)
        y = rng.uniform(*terrain.random_box_area_y)
        # 添加箱体几何
        ET.SubElement(
            body,
            "geom",
            {
                "name": f"random_box_{i:03d}",
                "type": "box",
                "pos": f"{x:.4f} {y:.4f} {height / 2.0:.4f}",
                "size": f"{size_x:.4f} {size_y:.4f} {height / 2.0:.4f}",
                "rgba": "0.38 0.44 0.48 1",
                "condim": "3",
                "friction": "1 0.01 0.01",
            },
        )
