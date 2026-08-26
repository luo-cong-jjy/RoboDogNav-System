"""Verify that the checked-in Nav2 map remains derived from the SDF world."""
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_factory_world_map_is_reproducible(tmp_path):
    output = tmp_path / 'factory_world_map.yaml'
    subprocess.run([
        str(ROOT / 'scripts' / 'gazebo_world_to_nav2_map.py'),
        str(ROOT / 'worlds' / 'factory_environment.world'),
        str(output),
    ], check=True)
    expected = ROOT / 'maps' / 'factory'
    assert output.read_bytes() == (expected / output.name).read_bytes()
    assert output.with_suffix('.pgm').read_bytes() == (
        expected / 'factory_world_map.pgm').read_bytes()
