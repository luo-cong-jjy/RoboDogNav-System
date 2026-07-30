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

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def test_vendor_and_ros_urdf_keep_official_kinematic_tree() -> None:
    vendor_root = ET.parse(PACKAGE_ROOT / 'urdf' / 'vendor' / 'M20.urdf').getroot()
    ros_root = ET.parse(PACKAGE_ROOT / 'urdf' / 'm20_official.urdf').getroot()
    vendor_links = [element.attrib['name'] for element in vendor_root.findall('link')]
    ros_links = [element.attrib['name'] for element in ros_root.findall('link')]
    vendor_joints = [element.attrib['name'] for element in vendor_root.findall('joint')]
    ros_joints = [element.attrib['name'] for element in ros_root.findall('joint')]
    assert vendor_links == ros_links
    assert vendor_joints == ros_joints
    assert len(vendor_links) == 17
    assert len(vendor_joints) == 16

    vendor_joint_data = [
        (
            joint.attrib,
            joint.find('origin').attrib,
            joint.find('parent').attrib,
            joint.find('child').attrib,
            joint.find('axis').attrib,
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
    assert ros_joint_data == vendor_joint_data


def test_ros_urdf_uses_package_mesh_uris() -> None:
    root = ET.parse(PACKAGE_ROOT / 'urdf' / 'm20_official.urdf').getroot()
    mesh_uris = [mesh.attrib['filename'] for mesh in root.iter('mesh')]
    assert len(mesh_uris) == 17
    assert all(
        uri.startswith('package://m20_official_description/meshes/')
        for uri in mesh_uris
    )


def test_compatibility_xacro_does_not_add_sensor_geometry() -> None:
    wrapper = (
        PACKAGE_ROOT / 'xacro' / 'm20.urdf.xacro'
    ).read_text(encoding='utf-8')
    assert '<xacro:include' in wrapper
    assert '<link ' not in wrapper
    assert '<joint ' not in wrapper
    assert 'lidar_link' not in wrapper
    assert 'imu_link' not in wrapper


def test_selected_official_mesh_hashes_are_fixed() -> None:
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
        assert _sha256(PACKAGE_ROOT / 'meshes' / name) == expected_hash
