import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'src' / 'm20_nav2_system' / 'scripts'
WORLD = ROOT / 'src' / 'm20_nav2_system' / 'worlds' / 'factory_environment.world'
TEMPLATE = ROOT / 'src' / 'm20_warehouse_system' / 'simulation' / 'm20_mujoco_backend' / 'models' / 'm20_robot.xml'


class FactoryWorldConversionTest(unittest.TestCase):
    def test_static_factory_scene_is_injected(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            geoms = directory / 'geoms.xml'
            output = directory / 'factory.xml'
            subprocess.run([
                'python3', str(SCRIPT / 'gazebo_world_to_mujoco_geoms.py'),
                str(WORLD), str(geoms),
            ], check=True)
            subprocess.run([
                'python3', str(SCRIPT / 'build_m20_mujoco_factory_world.py'),
                str(TEMPLATE), str(geoms), str(output),
            ], check=True)
            text = output.read_text(encoding='utf-8')
            self.assertEqual(text.count('name="gazebo_static_'), 10)
            self.assertEqual(text.count('<motor '), 16)
            self.assertIn('<freejoint name="floating_base"', text)


if __name__ == '__main__':
    unittest.main()
