# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify the isolated-workspace lock, patch, and preparation contract."""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SYSTEM_ROOT = PACKAGE_ROOT.parents[1]
SOURCE_ROOT = SYSTEM_ROOT.parent
PROJECT_PACKAGE_PATHS = {
    'm20_warehouse_interfaces': 'common/m20_warehouse_interfaces',
    'm20_multifloor_map': 'navigation/m20_multifloor_map',
    'm20_official_description': 'common/m20_official_description',
    'm20_warehouse_sim': 'simulation/m20_warehouse_sim',
    'm20_inspection_core': 'safety_mission/m20_inspection_core',
    'm20_scan_planner': 'navigation/m20_scan_planner',
    'm20_scan_navigation': 'navigation/m20_scan_navigation',
    'm20_locomotion_control': 'motion/m20_locomotion_control',
    'm20_mujoco_backend': 'simulation/m20_mujoco_backend',
    'm20_warehouse_inspection': 'bringup/m20_warehouse_inspection',
}


def _load_yaml(path: Path):
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_workspace_lock_matches_dependency_manifest() -> None:
    lock = _load_yaml(PACKAGE_ROOT / 'config' / 'workspace_lock.yaml')
    manifest = _load_yaml(PACKAGE_ROOT / 'dependencies.repos')
    sdk_manifest = _load_yaml(PACKAGE_ROOT / 'dependencies_sdk.repos')
    package_manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    assert package_manifest.findtext('version') == lock['project_release']
    scan = lock['source_dependencies']['scan_planner']
    model = lock['source_dependencies']['deep_robotics_model']
    sdk = lock['source_dependencies']['m20_sdk_deploy']
    assert manifest['repositories'][scan['path']]['url'] == scan['url']
    assert (
        manifest['repositories'][scan['path']]['version']
        == scan['revision']
    )
    assert manifest['repositories'][model['path']]['url'] == model['url']
    assert (
        manifest['repositories'][model['path']]['version']
        == model['revision']
    )
    assert sdk_manifest['repositories'][sdk['path']]['url'] == sdk['url']
    assert (
        sdk_manifest['repositories'][sdk['path']]['version']
        == sdk['revision']
    )


def test_frozen_rviz_v1_baseline_hashes_are_immutable() -> None:
    baseline = _load_yaml(
        PACKAGE_ROOT / 'config' / 'rviz_v1_baseline.yaml'
    )
    workspace_lock = _load_yaml(
        PACKAGE_ROOT / 'config' / 'workspace_lock.yaml'
    )
    package_manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    assert baseline['status'] == 'frozen'
    assert str(baseline['release']) == workspace_lock['project_release']
    assert str(baseline['release']) == package_manifest.findtext('version')

    for profile in baseline['stable_profiles'].values():
        config_path = PACKAGE_ROOT / profile['config']
        assert hashlib.sha256(config_path.read_bytes()).hexdigest() == (
            profile['config_sha256']
        )
        for map_asset in profile['maps']:
            assert 'pgm' not in map_asset
            assert 'pgm_sha256' not in map_asset
            for kind in ('pcd',):
                path = PACKAGE_ROOT / map_asset[kind]
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                assert digest == map_asset[f'{kind}_sha256']

    model = baseline['official_model']
    model_path = (
        SYSTEM_ROOT / PROJECT_PACKAGE_PATHS[model['package']] / model['path']
    )
    assert hashlib.sha256(model_path.read_bytes()).hexdigest() == (
        model['urdf_sha256']
    )
    assert 'config/route_challenge_system.yaml' in (
        baseline['excluded_from_baseline']
    )


def test_dependency_patch_checksum_is_locked() -> None:
    lock = _load_yaml(PACKAGE_ROOT / 'config' / 'workspace_lock.yaml')
    scan = lock['source_dependencies']['scan_planner']
    patch = PACKAGE_ROOT / scan['patch']
    digest = hashlib.sha256(patch.read_bytes()).hexdigest()
    assert digest == scan['patch_sha256']
    text = patch.read_text(encoding='utf-8')
    assert '-find_package(glm REQUIRED)' in text
    assert '+  find_package(glm REQUIRED)' in text
    assert '-  <depend>libglm-dev</depend>' in text
    integration_patch = PACKAGE_ROOT / scan['integration_patch']
    integration_digest = hashlib.sha256(
        integration_patch.read_bytes()
    ).hexdigest()
    assert integration_digest == scan['integration_patch_sha256']
    integration_text = integration_patch.read_text(encoding='utf-8')
    assert 'allow_global_map_updates' in integration_text
    assert 'toCompactPointCloud2' in integration_text
    assert 'reliable_cloud_transport' in integration_text
    sdk = lock['source_dependencies']['m20_sdk_deploy']
    sdk_patch = PACKAGE_ROOT / sdk['patch']
    sdk_digest = hashlib.sha256(sdk_patch.read_bytes()).hexdigest()
    assert sdk_digest == sdk['sdk_patch_sha256']
    sdk_text = sdk_patch.read_text(encoding='utf-8')
    assert 'rl_deploy_cmdvel' in sdk_text
    assert 'CmdVelInterface' in sdk_text


def test_all_locked_project_packages_exist_and_are_named_correctly() -> None:
    lock = _load_yaml(PACKAGE_ROOT / 'config' / 'workspace_lock.yaml')
    for package_name in lock['project_packages']:
        manifest = (
            SYSTEM_ROOT
            / PROJECT_PACKAGE_PATHS[package_name]
            / 'package.xml'
        )
        assert manifest.is_file()
        assert ET.parse(manifest).getroot().findtext('name') == package_name
    closure = lock['build_closure']
    assert len(closure) == 22
    assert set(lock['project_packages']).issubset(closure)
    assert {'drdds', 'm20_sdk_deploy'}.issubset(closure)
    for package_name in lock['excluded_project_packages']:
        package_root = SOURCE_ROOT / package_name
        assert (package_root / 'package.xml').is_file()
        assert (package_root / 'COLCON_IGNORE').is_file()


def test_prepare_script_is_non_destructive_and_verifies_revisions() -> None:
    lock = _load_yaml(PACKAGE_ROOT / 'config' / 'workspace_lock.yaml')
    script = (
        PACKAGE_ROOT / 'tools' / 'prepare_isolated_workspace.sh'
    ).read_text(encoding='utf-8')
    assert 'Target workspace must be empty' in script
    assert 'rm -' not in script
    assert 'git -C "${scan_target}" apply --check' in script
    assert 'integration_patch_file' in script
    assert (
        'apply --ignore-space-change --check '
        '"${integration_patch_file}"' in script
    )
    assert 'actual_scan_revision' in script
    assert 'actual_model_revision' in script
    assert 'actual_sdk_revision' in script
    assert lock['source_dependencies']['scan_planner']['revision'] in script
    assert (
        lock['source_dependencies']['deep_robotics_model']['revision']
        in script
    )
    assert (
        lock['source_dependencies']['m20_sdk_deploy']['revision']
        in script
    )
    assert 'patch checksum mismatch' in script
    assert 'Integration patch checksum mismatch' in script
    assert 'M20 SDK adapter patch checksum mismatch' in script
    assert 'git -C "${sdk_target}" apply --check' in script
    assert 'ignored_dependency_packages' in script
    assert 'deployed_message_source=' in script
    assert 'deep-robotics-msg' in script
    assert '--exclude \'COLCON_IGNORE\'' in script
    assert 'canonical_drdds_source=' not in script
    assert '"${sdk_target}/src/drdds"' in script
    assert 'historical top-level src/drdds copy is ignored' in script
    assert script.index('patch checksum mismatch') < script.index(
        'git -C "${scan_target}" apply --check'
    )


def test_cyclonedds_profile_keeps_pointcloud_transport_defaults() -> None:
    """The capacity profile must not force the WSL loopback data path."""
    text = (
        PACKAGE_ROOT / 'config' / 'cyclonedds_local.xml'
    ).read_text(encoding='utf-8')
    assert '<MaxAutoParticipantIndex>63</MaxAutoParticipantIndex>' in text
    assert '<NetworkInterface' not in text
    assert '<MaxMessageSize>' not in text
