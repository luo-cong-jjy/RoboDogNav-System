import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


class MujocoLaunchContractTest(unittest.TestCase):
    def test_factory_world_arguments_reach_backend(self):
        paths = [
            ROOT / 'src/m20_nav2_system/launch/m20_mujoco_navigation.launch.py',
            ROOT / 'src/m20_warehouse_system/bringup/m20_warehouse_inspection/launch/inspection_mission_mujoco.launch.py',
            ROOT / 'src/m20_warehouse_system/simulation/m20_mujoco_backend/launch/mujoco_backend.launch.py',
        ]
        for path in paths:
            text = path.read_text(encoding='utf-8')
            self.assertIn('world_source', text, path)
            self.assertIn('world_file', text, path)


if __name__ == '__main__':
    unittest.main()
