import unittest
import math
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SYSTEM_ROOT = PACKAGE.parents[1]


class MujocoLaunchContractTest(unittest.TestCase):
    def test_factory_world_arguments_reach_backend(self):
        paths = [
            PACKAGE / 'launch/m20_mujoco_navigation.launch.py',
            SYSTEM_ROOT / 'backend/m20_nav2_backend/launch/mujoco_backend.launch.py',
        ]
        for path in paths:
            text = path.read_text(encoding='utf-8')
            self.assertIn('world_source', text, path)
            self.assertIn('world_file', text, path)

    def test_gazebo_and_mujoco_use_isolated_worlds(self):
        gazebo_launch_files = [
            PACKAGE / 'launch/factory_saved_map_navigation.launch.py',
            PACKAGE / 'launch/factory_navigation.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_2d_scan.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py',
        ]
        for path in gazebo_launch_files:
            text = path.read_text(encoding='utf-8')
            self.assertIn('factory_environment.world', text, path)
            self.assertNotIn('factory_environment_mujoco.world', text, path)
            self.assertNotIn('factory_environment_v2.world', text, path)

        mujoco_launch_files = [
            PACKAGE / 'launch/m20_mujoco_navigation.launch.py',
            PACKAGE / 'launch/m20_mujoco_sensor_bridge.launch.py',
            PACKAGE / 'launch/m20_mujoco_viewer_only.launch.py',
            PACKAGE / 'launch/generate_factory_mujoco_world.launch.py',
        ]
        for path in mujoco_launch_files:
            text = path.read_text(encoding='utf-8')
            self.assertIn('factory_environment_mujoco.world', text, path)
            self.assertNotIn('factory_environment_v2.world', text, path)

        gazebo_world = ET.parse(
            PACKAGE / 'worlds/factory_environment.world').getroot()
        mujoco_world = ET.parse(
            PACKAGE / 'worlds/factory_environment_mujoco.world').getroot()

        obstacles = []
        for model in gazebo_world.findall('.//world/model'):
            trajectory = model.findtext('.//plugin/trajectory')
            cylinder = model.find('.//collision/geometry/cylinder')
            if trajectory and cylinder is not None:
                obstacles.append((model, cylinder, trajectory))

        self.assertEqual(len(obstacles), 5)
        for _, _, trajectory in obstacles:
            points = [
                [float(value) for value in waypoint.split()]
                for waypoint in trajectory.split(';')
            ]
            distance = math.hypot(
                points[1][1] - points[0][1],
                points[1][2] - points[0][2],
            )
            duration = points[1][0] - points[0][0]
            self.assertAlmostEqual(distance / duration, 0.35)

        self.assertEqual(
            gazebo_world.findall('.//world/actor'), [],
            'Gazebo dynamic navigation uses only scan-proxy cylinders',
        )

        obstacles = []
        for model in mujoco_world.findall('.//world/model'):
            trajectory = model.findtext('.//plugin/trajectory')
            cylinder = model.find('.//collision/geometry/cylinder')
            if trajectory and cylinder is not None:
                obstacles.append((model, cylinder, trajectory))

        self.assertEqual(len(obstacles), 5)
        self.assertTrue(all(
            float(cylinder.findtext('radius')) == 0.24
            for _, cylinder, _ in obstacles))
        self.assertTrue(all(
            float(cylinder.findtext('length')) == 1.5
            for _, cylinder, _ in obstacles))
        for _, _, trajectory in obstacles:
            points = [
                [float(value) for value in waypoint.split()]
                for waypoint in trajectory.split(';')
            ]
            distance = math.hypot(
                points[1][1] - points[0][1],
                points[1][2] - points[0][2],
            )
            duration = points[1][0] - points[0][0]
            self.assertAlmostEqual(distance / duration, 1.0)

        self.assertEqual(len(mujoco_world.findall('.//world/actor')), 5)

    def test_mujoco_0932_baseline_restores_dynamic_prediction(self):
        bridge = (
            PACKAGE / 'launch/m20_mujoco_sensor_bridge.launch.py'
        ).read_text(encoding='utf-8')
        lidar = (
            PACKAGE / 'scripts/m20_grid_lidar_simulator'
        ).read_text(encoding='utf-8')
        markers = (
            PACKAGE / 'scripts/m20_factory_scene_markers'
        ).read_text(encoding='utf-8')

        for value in (
            "'dynamic_prediction_horizon': 7.0",
            "'dynamic_prediction_max_horizon': 8.0",
            "'dynamic_prediction_samples': 8",
            "'dynamic_prediction_lead_time': 3.5",
            "'dynamic_prediction_tail_time': 0.25",
            "'robot_prediction_radius': 0.68",
            "'dynamic_prediction_margin': 0.35",
            "'prediction_base_distance': 2.5",
            "'prediction_reaction_time': 7.0",
            "'prediction_max_distance': 9.0",
        ):
            self.assertIn(value, bridge)
        self.assertNotIn('dynamic_time_origin', bridge)
        self.assertIn("'/scan_predicted'", lidar)
        self.assertIn("'/m20/factory/predicted_obstacles'", lidar)
        self.assertIn('_predicted_dynamic_positions', lidar)
        self.assertIn('_publish_predicted_markers', lidar)
        self.assertIn('time.time()', lidar)
        self.assertIn('time.time()', markers)

    def test_lidar_prediction_reuses_static_scan(self):
        lidar = (PACKAGE / 'scripts/m20_grid_lidar_simulator').read_text(
            encoding='utf-8')
        self.assertIn('The static map is identical for both scans', lidar)
        self.assertIn('def _dynamic_range(', lidar)
        self.assertIn('for measured, angle in zip(', lidar)

    def test_odom_adapter_handles_shutdown_conversion_race(self):
        adapter = (PACKAGE / 'scripts/m20_mujoco_odom_adapter').read_text(
            encoding='utf-8')
        self.assertIn('except RuntimeError as exc:', adapter)
        self.assertIn("if rclpy.ok():", adapter)

    def test_sensor_nodes_exit_cleanly_on_ctrl_c(self):
        for name in ('m20_grid_lidar_simulator', 'm20_factory_scene_markers'):
            source = (PACKAGE / 'scripts' / name).read_text(encoding='utf-8')
            self.assertIn('except KeyboardInterrupt:', source, name)


if __name__ == '__main__':
    unittest.main()
