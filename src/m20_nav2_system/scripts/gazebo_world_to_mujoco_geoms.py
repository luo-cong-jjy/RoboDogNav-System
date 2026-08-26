#!/usr/bin/env python3
"""Extract static SDF box/cylinder collisions as MuJoCo world geoms."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

def vec(text, n=6):
    values = [float(v) for v in (text or '').split()]
    return (values + [0.0] * n)[:n]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('world', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = ET.parse(args.world).getroot()
    lines = ['<!-- generated from SDF; static collisions only -->', '<mujoco>', '  <worldbody>']
    count = 0
    skipped_dynamic = []
    for model in root.findall('.//world/model'):
        if model.findtext('static', 'false').strip().lower() != 'true':
            skipped_dynamic.append(model.attrib.get('name', '<unnamed>'))
            continue
        mx, my, mz, mr, mp, myaw = vec(model.findtext('pose'))
        for collision in model.findall('.//collision'):
            geometry = collision.find('geometry')
            if geometry is None:
                continue
            cx, cy, cz, cr, cp, cyaw = vec(collision.findtext('pose'))
            position = f'{mx + cx:.6f} {my + cy:.6f} {mz + cz:.6f}'
            rotation = f'{mr + cr:.6f} {mp + cp:.6f} {myaw + cyaw:.6f}'
            name = f'gazebo_static_{count}'
            box = geometry.find('box/size')
            cylinder = geometry.find('cylinder')
            if box is not None:
                sx, sy, sz = vec(box.text, 3)
                lines.append(f'    <geom name="{name}" type="box" pos="{position}" euler="{rotation}" size="{sx/2:.6f} {sy/2:.6f} {sz/2:.6f}"/>')
                count += 1
            elif cylinder is not None:
                radius = float(cylinder.findtext('radius', '0'))
                length = float(cylinder.findtext('length', '0'))
                lines.append(f'    <geom name="{name}" type="cylinder" pos="{position}" euler="{rotation}" size="{radius:.6f} {length/2:.6f}"/>')
                count += 1
    lines += ['  </worldbody>', '</mujoco>', '']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(lines), encoding='utf-8')
    print(f'generated {count} geoms -> {args.output}')
    if skipped_dynamic:
        print('skipped dynamic models: ' + ', '.join(skipped_dynamic))

if __name__ == '__main__':
    main()
