# Copyright (c) 2026 Virdyn Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Verify official asset provenance and wrapper-only changes."""
# 文件用途：测试脚本，验证官方模型资源来源正确且封装只做"包装级"改动
# 核心检查点：
#   1. ROS 侧 URDF 与 vendor 官方 URDF 的运动学树（link/joint）完全一致
#   2. ROS 侧 URDF 的 mesh 路径全部改为包内 package:// URI
#   3. 兼容 xacro 未添加任何传感器/几何
#   4. 选定的官方 mesh 文件哈希与预期一致（防止资源被篡改）

import hashlib        # 计算文件 SHA-256 哈希（资源完整性校验）
from pathlib import Path  # 路径处理
import xml.etree.ElementTree as ET  # 解析 URDF/XML


PACKAGE_ROOT = Path(__file__).resolve().parents[1]  # 包根目录（test/ 的上一级）


def _sha256(path: Path) -> str:
    """计算文件 SHA-256 十六进制摘要。"""
    digest = hashlib.sha256()
    digest.update(path.read_bytes())  # 按字节读取整个文件
    return digest.hexdigest()


def test_vendor_and_ros_urdf_keep_official_kinematic_tree() -> None:
    """测试：vendor 官方 URDF 与 ROS 侧 URDF 的运动学树必须一致。"""
    vendor_root = ET.parse(PACKAGE_ROOT / 'urdf' / 'vendor' / 'M20.urdf').getroot()   # 解析官方原版
    ros_root = ET.parse(PACKAGE_ROOT / 'urdf' / 'm20_official.urdf').getroot()        # 解析 ROS 封装版
    vendor_links = [element.attrib['name'] for element in vendor_root.findall('link')]   # 官方 link 名列表
    ros_links = [element.attrib['name'] for element in ros_root.findall('link')]        # ROS 版 link 名列表
    vendor_joints = [element.attrib['name'] for element in vendor_root.findall('joint')] # 官方 joint 名列表
    ros_joints = [element.attrib['name'] for element in ros_root.findall('joint')]      # ROS 版 joint 名列表
    assert vendor_links == ros_links      # link 集合必须完全相同
    assert vendor_joints == ros_joints    # joint 集合必须完全相同
    assert len(vendor_links) == 17        # M20 模型共 17 个 link
    assert len(vendor_joints) == 16       # 共 16 个 joint

    # 提取每个 joint 的完整属性元组（类型/原点/父子/转轴）
    vendor_joint_data = [
        (
            joint.attrib,                       # joint 属性（type/name）
            joint.find('origin').attrib,        # 原点（xyz/rpy）
            joint.find('parent').attrib,        # 父 link
            joint.find('child').attrib,         # 子 link
            joint.find('axis').attrib,          # 旋转轴
        )
        for joint in vendor_root.findall('joint')
    ]
    ros_joint_data = [
        (
            joint.attrib,
            joint.find('origin').attrib,
            joint.find('parent').attrib,
            joint.find('child').attrib,
            joint.find('axis').attrib,
        )
        for joint in ros_root.findall('joint')
    ]
    # Origins, axes, parent/child relationships, and joint types are copied
    # unchanged, which prevents a detached body/leg display.
    # （原点/转轴/父子关系/关节类型必须逐项一致，防止模型部件悬空显示）
    assert ros_joint_data == vendor_joint_data


def test_ros_urdf_uses_package_mesh_uris() -> None:
    """测试：ROS 侧 URDF 的 mesh 路径全部为包内 package:// URI。"""
    root = ET.parse(PACKAGE_ROOT / 'urdf' / 'm20_official.urdf').getroot()
    mesh_uris = [mesh.attrib['filename'] for mesh in root.iter('mesh')]  # 收集所有 mesh 引用
    assert len(mesh_uris) == 17  # 每个 link 一个 mesh，共 17 个
    assert all(
        uri.startswith('package://m20_official_description/meshes/')  # 必须指向本包 meshes 目录
        for uri in mesh_uris
    )


def test_compatibility_xacro_does_not_add_sensor_geometry() -> None:
    """测试：兼容 xacro 不添加任何传感器/几何。"""
    wrapper = (
        PACKAGE_ROOT / 'xacro' / 'm20.urdf.xacro'
    ).read_text(encoding='utf-8')
    assert '<xacro:include' in wrapper  # 应通过 include 引入官方 URDF
    assert '<link ' not in wrapper      # 不允许新增 link
    assert '<joint ' not in wrapper     # 不允许新增 joint
    assert 'lidar_link' not in wrapper  # 不允许添加激光雷达 link
    assert 'imu_link' not in wrapper    # 不允许添加 IMU link


def test_selected_official_mesh_hashes_are_fixed() -> None:
    """测试：选定官方 mesh 文件的 SHA-256 哈希必须与预期一致（防篡改）。"""
    expected = {
        'base_link.STL': (
            'a7984137d06782f513858fd699111109978301a9dff53ff37abf5413b922247f'
        ),
        'fl_wheel.STL': (
            'bcfca68aeeb8e5f118d4623a384cb040f611c32c1115b8574aed4c546e64685d'
        ),
        'hr_knee.STL': (
            '2fcf688c30596b3640e528defa92b58fe7804586586a681677115ea6d93f60e3'
        ),
    }
    for name, expected_hash in expected.items():
        assert _sha256(PACKAGE_ROOT / 'meshes' / name) == expected_hash  # 实际哈希必须匹配
