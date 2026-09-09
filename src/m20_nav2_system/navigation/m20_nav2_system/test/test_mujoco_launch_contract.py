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
            PACKAGE / 'launch/factory_slam_navigation.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_2d_scan.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py',
        ]
        for path in gazebo_launch_files:
            text = path.read_text(encoding='utf-8')
            if path.name in ('factory_saved_map_navigation.launch.py',
                             'factory_navigation.launch.py'):
                self.assertIn('factory_environment_gazebo.world', text, path)
            elif path.name == 'factory_slam_navigation.launch.py':
                self.assertIn('factory_environment_gazebo_mapping.world', text,
                              path)
            else:
                self.assertIn('factory_environment_gazebo.world', text, path)
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
            PACKAGE / 'worlds/factory_environment_gazebo.world').getroot()
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

        # The files are independent, but their effective worker trajectories
        # must remain equivalent: Gazebo bakes the 0.35 speed into its time
        # stamps while MuJoCo applies dynamic_speed_scale=0.35 at runtime.
        gazebo_by_name = {
            model.attrib['name']: model.findtext('.//plugin/trajectory')
            for model in gazebo_world.findall('.//world/model')
            if model.findtext('.//plugin/trajectory')
        }
        mujoco_by_name = {
            model.attrib['name']: model.findtext('.//plugin/trajectory')
            for model in mujoco_world.findall('.//world/model')
            if model.findtext('.//plugin/trajectory')
        }
        self.assertEqual(set(gazebo_by_name), set(mujoco_by_name))
        for name in gazebo_by_name:
            gazebo_points = [
                [float(value) for value in waypoint.split()]
                for waypoint in gazebo_by_name[name].split(';')
            ]
            mujoco_points = [
                [float(value) for value in waypoint.split()]
                for waypoint in mujoco_by_name[name].split(';')
            ]
            self.assertEqual(len(gazebo_points), len(mujoco_points))
            for gazebo_point, mujoco_point in zip(gazebo_points, mujoco_points):
                self.assertAlmostEqual(gazebo_point[0] * 0.35,
                                       mujoco_point[0], places=5)
                for gazebo_value, mujoco_value in zip(gazebo_point[1:],
                                                       mujoco_point[1:]):
                    self.assertAlmostEqual(gazebo_value, mujoco_value,
                                           places=5)

    def test_backend_profiles_do_not_cross_reference(self):
        gazebo_launches = (
            PACKAGE / 'launch/factory_navigation.launch.py',
            PACKAGE / 'launch/factory_saved_map_navigation.launch.py',
            PACKAGE / 'launch/factory_slam_navigation.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_2d_scan.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py',
            PACKAGE / 'launch/nav2_map_navigation_m20_factory.launch.py',
            PACKAGE / 'launch/factory_inspection_mission_gazebo.launch.py',
        )
        for path in gazebo_launches:
            text = path.read_text(encoding='utf-8')
            self.assertNotIn('m20_mujoco', text, path)
            self.assertNotIn('factory_environment_mujoco.world', text, path)
            self.assertNotIn('nav2_params.yaml', text, path)
            self.assertNotIn('nav2_sandbox.rviz', text, path)
            self.assertNotIn('m20_grid_lidar_simulator', text, path)

        factory_navigation = (
            PACKAGE / 'launch/factory_navigation.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn('_factory_navigation_use_rviz', factory_navigation)
        self.assertIn('SetLaunchConfiguration', factory_navigation)

        for path in (
            PACKAGE / 'models/m20_gazebo_2d_scan.urdf',
            PACKAGE / 'models/m20_gazebo_3d_rslidar.urdf',
        ):
            text = path.read_text(encoding='utf-8')
            self.assertIn('package://m20_nav2_system/models/meshes/', text, path)
            self.assertNotIn('model://m20_nav2_system/models/meshes/', text, path)

        # Gazebo Classic rewrites package:// mesh URIs while importing URDF.
        # The Gazebo launch files therefore spawn an absolute-path runtime
        # copy, while the package URDF remains portable for RViz/TF publishing.
        for path in (
            PACKAGE / 'launch/gazebo_sensor_m20_factory_2d_scan.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py',
        ):
            text = path.read_text(encoding='utf-8')
            self.assertIn('_make_gazebo_spawn_urdf', text, path)
            self.assertIn('gazebo_robot_source_urdf', text, path)
            self.assertNotIn('GAZEBO_MODEL_PATH', text, path)

        gazebo_params = (
            PACKAGE / 'config/nav2_params_gazebo.yaml'
        ).read_text(encoding='utf-8')
        mujoco_params = (
            PACKAGE / 'config/nav2_params_mujoco.yaml'
        ).read_text(encoding='utf-8')
        self.assertIn('observation_sources: scan', gazebo_params)
        self.assertIn('observation_sources: scan predicted_scan',
                       gazebo_params)
        self.assertIn('topic: /scan_predicted_gazebo', gazebo_params)
        self.assertEqual(gazebo_params.count('observation_sources: scan\n'), 1)
        self.assertIn('use_rotate_to_heading: true', gazebo_params)
        self.assertIn('rotate_to_heading_min_angle: 0.785', gazebo_params)
        self.assertIn('topic: /scan_predicted', mujoco_params)

        inspection = (
            PACKAGE / 'launch/factory_inspection_mission_gazebo.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn('factory_inspection_midpoints_gazebo.yaml', inspection)
        self.assertIn('factory_inspection_nav2_mission_gazebo', inspection)

        for path in (
            PACKAGE / 'launch/gazebo_sensor_m20_factory_2d_scan.launch.py',
            PACKAGE / 'launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py',
        ):
            text = path.read_text(encoding='utf-8')
            self.assertIn('rslidar_pointcloud_to_scan_gazebo.launch.py', text)
            self.assertIn('m20_standing_joint_state_publisher_gazebo', text)
            self.assertIn('goal_pose_restamper_gazebo', text)

        sensor_2d = (
            PACKAGE / 'launch/gazebo_sensor_m20_factory_2d_scan.launch.py'
        ).read_text(encoding='utf-8')
        sensor_3d = (
            PACKAGE / 'launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn('m20_gazebo_2d_scan.urdf', sensor_2d)
        self.assertIn('m20_gazebo_3d_rslidar.urdf', sensor_3d)
        self.assertIn('M20_gazebo_visual.urdf', sensor_2d)
        self.assertIn('M20_gazebo_visual.urdf', sensor_3d)
        self.assertIn('enable_dynamic_tracker', sensor_2d)
        self.assertIn('m20_gazebo_dynamic_obstacle_tracker', sensor_2d)
        self.assertIn('enable_dynamic_tracker', sensor_3d)
        self.assertIn('m20_gazebo_dynamic_obstacle_tracker', sensor_3d)

        saved_navigation = (
            PACKAGE / 'launch/factory_saved_map_navigation.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn('"enable_dynamic_tracker": "true"', saved_navigation)

        slam_navigation = (
            PACKAGE / 'launch/factory_slam_navigation.launch.py'
        ).read_text(encoding='utf-8')
        self.assertNotIn('"enable_dynamic_tracker": "true"', slam_navigation)

        keyboard = (
            PACKAGE / 'launch/factory_slam_navigation.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn('m20_keyboard_teleop_gazebo', keyboard)

        gazebo_scripts = (
            'goal_pose_restamper_gazebo',
            'm20_standing_joint_state_publisher_gazebo',
            'm20_keyboard_teleop_gazebo',
            'factory_inspection_nav2_mission_gazebo',
            'm20_gazebo_dynamic_obstacle_tracker',
        )
        for name in gazebo_scripts:
            self.assertTrue((PACKAGE / 'scripts' / name).is_file(), name)

        tracker = (
            PACKAGE / 'scripts/m20_gazebo_dynamic_obstacle_tracker'
        ).read_text(encoding='utf-8')
        self.assertIn("'/scan'", tracker)
        self.assertIn("'/map'", tracker)
        self.assertIn("'/odom'", tracker)
        self.assertNotIn('world_file', tracker)
        self.assertNotIn('m20_grid_lidar_simulator', tracker)

        mujoco_launches = (
            PACKAGE / 'launch/m20_mujoco_navigation.launch.py',
            PACKAGE / 'launch/m20_mujoco_sensor_bridge.launch.py',
            PACKAGE / 'launch/m20_mujoco_static_navigation.launch.py',
            PACKAGE / 'launch/m20_mujoco_viewer_only.launch.py',
        )
        for path in mujoco_launches:
            text = path.read_text(encoding='utf-8')
            self.assertNotIn('factory_environment_gazebo', text, path)
            self.assertNotIn('nav2_params_gazebo.yaml', text, path)
            self.assertNotIn('rslidar_pointcloud_to_scan_gazebo', text, path)
            self.assertNotIn('goal_pose_restamper_gazebo', text, path)

        mujoco_params = (
            PACKAGE / 'config/nav2_params_mujoco.yaml'
        ).read_text(encoding='utf-8')
        self.assertIn('topic: /scan_predicted', mujoco_params)

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
