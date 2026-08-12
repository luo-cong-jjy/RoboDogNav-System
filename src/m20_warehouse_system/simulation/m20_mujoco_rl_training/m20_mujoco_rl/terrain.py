"""MJCF terrain generation for M20 MuJoCo training."""

from __future__ import annotations

import hashlib
import json
import tempfile
from dataclasses import asdict
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .config import TerrainConfig, default_m20_mjcf_dir


def _base_m20_xml() -> Path:
    return default_m20_mjcf_dir() / "M20.xml"


def _official_stair_xml() -> Path:
    return default_m20_mjcf_dir() / "M20_stair.xml"


def resolve_model_xml(model_xml: str | None, terrain: TerrainConfig) -> Path:
    if model_xml is not None:
        return Path(model_xml).expanduser().resolve()
    if terrain.name == "stair_official":
        return _official_stair_xml().resolve()
    return build_generated_xml(terrain)


def build_generated_xml(terrain: TerrainConfig) -> Path:
    base_xml = _base_m20_xml().resolve()
    tree = ET.parse(base_xml)
    root = tree.getroot()

    mesh_dir = base_xml.parents[1] / "meshes"
    for compiler in root.findall("compiler"):
        if "meshdir" in compiler.attrib:
            compiler.set("meshdir", str(mesh_dir.resolve()))

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError(f"Cannot find <worldbody> in {base_xml}")

    if terrain.name == "flat":
        pass
    elif terrain.name == "stair_easy":
        _add_stairs(worldbody, terrain)
    elif terrain.name == "random_boxes":
        _add_random_boxes(worldbody, terrain)
    else:
        raise ValueError(
            f"Unsupported generated terrain: {terrain.name}. "
            "Use flat, stair_easy, random_boxes or stair_official."
        )

    out_dir = Path(tempfile.gettempdir()) / "m20_mujoco_rl_training"
    out_dir.mkdir(parents=True, exist_ok=True)
    terrain_key = json.dumps(asdict(terrain), sort_keys=True, separators=(",", ":"))
    terrain_hash = hashlib.sha1(terrain_key.encode("utf-8")).hexdigest()[:10]
    out_path = out_dir / f"M20_{terrain.name}_{terrain.seed}_{terrain_hash}.xml"
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path


def _add_stairs(worldbody: ET.Element, terrain: TerrainConfig) -> None:
    stair_body = ET.SubElement(worldbody, "body", {"name": "training_staircase", "pos": "0 0 0"})
    for i in range(terrain.stair_count):
        top = terrain.stair_height * (i + 1)
        center_x = terrain.stair_start_x + i * terrain.stair_depth + terrain.stair_depth / 2.0
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


def _add_random_boxes(worldbody: ET.Element, terrain: TerrainConfig) -> None:
    rng = np.random.default_rng(terrain.seed)
    body = ET.SubElement(worldbody, "body", {"name": "training_random_boxes", "pos": "0 0 0"})
    for i in range(terrain.random_box_count):
        height = rng.uniform(*terrain.random_box_height)
        size_x = rng.uniform(0.08, 0.25)
        size_y = rng.uniform(0.08, 0.25)
        x = rng.uniform(*terrain.random_box_area_x)
        y = rng.uniform(*terrain.random_box_area_y)
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
