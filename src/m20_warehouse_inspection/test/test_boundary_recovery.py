# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit and composition guards for fixed boundary-recovery validation."""

from pathlib import Path

import numpy as np
import yaml

from m20_scan_navigation.grid_route import (
    classify_continuous_transitions,
    clearance_from_blocked,
    clear_boundary_gateway_inflation,
    compress_route_for_scan,
    GridGeometry,
    make_blocked_grid,
    plan_metric_route,
)
from m20_warehouse_inspection.boundary_recovery_probe import (
    aggregate_reports,
    build_report,
    fallback_scan_goals,
)
from m20_warehouse_inspection.map_assets import read_pgm


ROOT = Path(__file__).parents[1]
SOURCE = ROOT.parent


def _sample(elapsed, x, contact_events=0, inside=False):
    return {
        'elapsed_sec': elapsed,
        'x': x,
        'y': 0.0,
        'yaw': 0.0,
        'obstacle_body_clearance_m': 0.40,
        'scan_inflation_clearance_m': -0.01 if inside else 0.15,
        'guard_clearance_m': 0.10,
        'inside_scan_inflation': inside,
        'obstacle_contact_event_count': contact_events,
        'obstacle_contact_peak_force_n': 0.0,
        'obstacle_contact_pairs': '',
    }


def test_fallback_goals_exclude_direct_goal_and_keep_first_routed_goal():
    goals = [
        {'elapsed_sec': 0.05, 'x': -36.0},
        {'elapsed_sec': 7.92, 'x': -8.0},
        {'elapsed_sec': 12.0, 'x': -20.0},
    ]
    states = [{'elapsed_sec': 8.0, 'state': 'TRACKING_ROUTE'}]
    assert fallback_scan_goals(goals, states) == goals[1:]
    assert fallback_scan_goals(goals, []) == []


def test_report_accepts_contact_free_stop_to_stop_recovery():
    report = build_report(
        case_name='upper_boundary',
        samples=[
            _sample(0.0, -7.2, inside=True),
            _sample(0.5, -7.4),
            _sample(10.0, -36.0),
        ],
        target=(-36.0, 0.0),
        action_success=True,
        action_error_code=0,
        action_message='goal reached',
        guard_events=[
            'RECOVERY_BUDGET_EXHAUSTED:test',
            'MAP_EDGE_INWARD_RECOVERY; trigger=test',
        ],
        route_states=[{'elapsed_sec': 1.0, 'state': 'TRACKING_ROUTE'}],
        scan_goals=[
            {
                'elapsed_sec': 1.0,
                'x': -8.0,
                'y': 0.0,
                'stationary_before_publish_sec': 0.0,
            },
            {
                'elapsed_sec': 4.0,
                'x': -20.0,
                'y': 0.0,
                'stationary_before_publish_sec': 0.31,
            },
        ],
        feedback=[],
        backend_faults=[],
        stop_stable_sec=0.30,
    )
    assert report['accepted'] is True
    assert report['fallback_triggered'] is True
    assert report['scan_inflated_region_sample_count'] == 1
    assert report['recovery_budget_exhausted_event_count'] == 1
    assert report['map_edge_inward_recovery_event_count'] == 1
    assert report['intermediate_stop_violation_count'] == 0


def test_report_accepts_classified_continuous_handoff_without_stop():
    report = build_report(
        case_name='narrow_corridor_entry',
        samples=[_sample(0.0, -37.0), _sample(8.0, -30.62)],
        target=(-30.62, 0.0),
        action_success=True,
        action_error_code=0,
        action_message='goal reached',
        guard_events=[],
        route_states=[
            {'elapsed_sec': 0.5, 'state': 'TRACKING_ROUTE'},
        ],
        scan_goals=[
            {
                'elapsed_sec': 0.5,
                'x': -35.0,
                'y': 0.0,
                'stationary_before_publish_sec': 0.0,
                'stop_required_before_publish': False,
            },
            {
                'elapsed_sec': 3.0,
                'x': -32.0,
                'y': 0.0,
                'stationary_before_publish_sec': 0.0,
                'stop_required_before_publish': False,
            },
        ],
        feedback=[],
        backend_faults=[],
        stop_stable_sec=0.30,
    )
    assert report['accepted'] is True
    assert report['continuous_handoff_count'] == 1
    assert report['intermediate_stop_check_count'] == 0
    assert report['intermediate_stop_violation_count'] == 0


def test_dense_narrow_entry_keeps_route_and_removes_artificial_stops():
    config = yaml.safe_load(
        (ROOT / 'config' / 'dense_four_corner_system.yaml').read_text(
            encoding='utf-8'
        )
    )
    floor = config['floors']['F1']
    occupancy_yaml = ROOT / floor['occupancy_file']
    description = yaml.safe_load(
        occupancy_yaml.read_text(encoding='utf-8')
    )
    image = read_pgm(occupancy_yaml.parent / description['image'])
    source_order = np.flipud(image)
    occupancy = np.full(source_order.shape, -1, dtype=np.int8)
    occupancy[source_order >= 250] = 0
    occupancy[source_order <= 10] = 100
    offset = floor['simulation_offset']
    geometry = GridGeometry(
        image.shape[1],
        image.shape[0],
        float(description['resolution']),
        float(description['origin'][0] + offset[0]),
        float(description['origin'][1] + offset[1]),
    )
    blocked = make_blocked_grid(
        occupancy.ravel(), geometry, 50, 0.60
    )
    blocked = clear_boundary_gateway_inflation(
        blocked, occupancy.ravel(), geometry, 50, 0.0, 0.0, 4.0, 0.60
    )
    clearance = clearance_from_blocked(blocked)
    start = (-37.0, 0.0)
    route = plan_metric_route(
        blocked,
        geometry,
        start,
        (-30.62, 3.18),
        1.50,
        0.90,
        0.20,
        4.0,
        clearance,
    )
    subgoals = compress_route_for_scan(
        route, blocked, geometry, clearance, 0.20
    )
    transitions = classify_continuous_transitions(
        start,
        subgoals,
        blocked,
        geometry,
        clearance,
        0.70,
        0.54,
        0.70,
        0.10,
    )

    assert len(route) == 14
    assert len(subgoals) == 6
    assert transitions == [True, True, True, True, True, False]


def test_report_rejects_physical_contact_or_rolling_subgoal():
    report = build_report(
        case_name='shared_origin_exit',
        samples=[_sample(0.0, 0.1), _sample(2.0, -37.0, contact_events=1)],
        target=(-37.0, 0.0),
        action_success=True,
        action_error_code=0,
        action_message='goal reached',
        guard_events=[],
        route_states=[
            {'elapsed_sec': 0.5, 'state': 'TRACKING_ROUTE'},
        ],
        scan_goals=[
            {
                'elapsed_sec': 0.5,
                'x': -1.0,
                'y': 0.0,
                'stationary_before_publish_sec': 0.0,
            },
            {
                'elapsed_sec': 1.0,
                'x': -10.0,
                'y': 0.0,
                'stationary_before_publish_sec': 0.10,
            },
        ],
        feedback=[],
        backend_faults=[],
        stop_stable_sec=0.30,
    )
    assert report['accepted'] is False
    assert report['obstacle_contact_event_count'] == 1
    assert report['intermediate_stop_violation_count'] == 1


def test_aggregate_keeps_case_level_risk_counts():
    base = {
        'duration_sec': 4.0,
        'action_success': True,
        'obstacle_contact_event_count': 0,
        'fallback_triggered': True,
        'map_edge_inward_recovery_event_count': 1,
        'scan_inflated_region_sample_count': 1,
        'intermediate_stop_violation_count': 0,
        'minimum_scan_inflation_clearance_m': -0.02,
        'longest_scan_inflated_region_sec': 0.4,
    }
    reports = [
        {**base, 'case': 'upper_boundary', 'accepted': True},
        {**base, 'case': 'upper_boundary', 'accepted': True},
        {
            **base,
            'case': 'shared_origin_exit',
            'accepted': False,
            'action_success': False,
        },
    ]
    summary = aggregate_reports(reports)
    assert summary['independent_cold_starts'] == 3
    assert summary['accepted_runs'] == 2
    assert summary['all_runs_accepted'] is False
    assert summary['cases']['upper_boundary']['observed_success_rate'] == 1.0
    assert summary['cases']['upper_boundary'][
        'map_edge_inward_recovery_runs'
    ] == 2
    assert summary['cases']['shared_origin_exit']['accepted_runs'] == 0


def test_launch_uses_simulation_only_cold_start_overrides():
    launch = (
        ROOT / 'launch' / 'boundary_recovery_mujoco.launch.py'
    ).read_text(encoding='utf-8')
    main_launch = (
        ROOT / 'launch' / 'inspection_mission_mujoco.launch.py'
    ).read_text(encoding='utf-8')
    backend_launch = (
        SOURCE
        / 'm20_mujoco_backend'
        / 'launch'
        / 'mujoco_backend.launch.py'
    ).read_text(encoding='utf-8')
    backend_source = (
        SOURCE
        / 'm20_mujoco_backend'
        / 'm20_mujoco_backend'
        / 'backend_node.py'
    ).read_text(encoding='utf-8')

    assert "'upper_boundary'" in launch
    assert "'shared_origin_exit'" in launch
    assert "'rear_corridor'" in launch
    assert "'narrow_corridor_entry'" in launch
    assert "'use_grid_route': LaunchConfiguration('use_grid_route')" in launch
    assert "'initial': (-36.0, 0.0, 0.0)" in launch
    assert 'inspection_mission_mujoco.launch.py' in launch
    assert "executable='m20_boundary_recovery_probe'" in launch
    assert "'initial_x': initial_x" in main_launch
    assert "initial_component('initial_x', 0)" in backend_launch
    assert 'SetSimulationPose' not in backend_source
    probe = (
        ROOT
        / 'm20_warehouse_inspection'
        / 'boundary_recovery_probe.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/navigation/scan_goal_internal'" in probe
