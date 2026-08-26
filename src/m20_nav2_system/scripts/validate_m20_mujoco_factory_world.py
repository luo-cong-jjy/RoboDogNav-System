#!/usr/bin/env python3
"""Validate a generated M20 factory MJCF without requiring MuJoCo Python."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mjcf', type=Path)
    parser.add_argument('--expected-static-geoms', type=int, required=True)
    args = parser.parse_args()
    root = ET.parse(args.mjcf).getroot()
    actuators = root.findall('.//actuator/*')
    static_geoms = [g for g in root.findall('.//geom')
                    if g.attrib.get('name', '').startswith('gazebo_static_')]
    names = {g.attrib.get('name') for g in static_geoms}
    if len(actuators) != 16:
        raise SystemExit(f'expected 16 actuators, got {len(actuators)}')
    if len(static_geoms) != args.expected_static_geoms:
        raise SystemExit(
            f'expected {args.expected_static_geoms} static geoms, got {len(static_geoms)}')
    if len(names) != len(static_geoms):
        raise SystemExit('duplicate generated geom names')
    if not root.findall('.//joint[@type="free"]') and not root.findall('.//freejoint'):
        raise SystemExit('floating base is missing')
    if not root.findall('.//sensor/gyro') or not root.findall('.//sensor/accelerometer'):
        raise SystemExit('IMU sensors are missing')
    print(f'valid M20 factory MJCF: actuators=16 static_geoms={len(static_geoms)}')


if __name__ == '__main__':
    main()
