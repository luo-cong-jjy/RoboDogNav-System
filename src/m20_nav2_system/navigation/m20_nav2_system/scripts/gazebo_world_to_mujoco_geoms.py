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
    lines = ['<!-- generated from the Nav2 factory SDF -->', '<mujoco>', '  <worldbody>']
    count = 0
    dynamic_count = 0
    for model in root.findall('.//world/model'):
        # The official M20 MJCF template already owns the physical/rendered
        # floor plane at z=0.  Converting the SDF `ground` box as well puts a
        # second, nearly coplanar surface at z=0 and causes OpenGL z-fighting
        # (the apparent translucent, flickering membrane in the viewer).
        model_name = model.attrib.get('name', '').strip().lower()
        if model_name in {'ground', 'ground_plane', 'floor'}:
            continue
        is_static = model.findtext('static', 'false').strip().lower() == 'true'
        mx, my, mz, mr, mp, myaw = vec(model.findtext('pose'))
        for collision in model.findall('.//collision'):
            geometry = collision.find('geometry')
            if geometry is None:
                continue
            cx, cy, cz, cr, cp, cyaw = vec(collision.findtext('pose'))
            position = f'{mx + cx:.6f} {my + cy:.6f} {mz + cz:.6f}'
            rotation = f'{mr + cr:.6f} {mp + cp:.6f} {myaw + cyaw:.6f}'
            name = f'gazebo_{"static" if is_static else "dynamic_visual"}_{count}'
            # Static objects participate in MuJoCo collision. Dynamic workers
            # remain visual-only here: their authoritative motion and sensing
            # are provided by the RViz/Nav2 simulator, not MuJoCo physics.
            common = (' group="0" rgba="0.32 0.42 0.52 1.0"'
                      if is_static else
                      ' group="0" rgba="0.90 0.12 0.10 1.0" contype="0" conaffinity="0"')
            box = geometry.find('box/size')
            cylinder = geometry.find('cylinder')
            if box is not None:
                sx, sy, sz = vec(box.text, 3)
                geom = f'<geom name="{name}" type="box" pos="{position}" euler="{rotation}" size="{sx/2:.6f} {sy/2:.6f} {sz/2:.6f}"{common}/>'
                lines.append(f'    {geom}')
                count += 1
            elif cylinder is not None:
                radius = float(cylinder.findtext('radius', '0'))
                length = float(cylinder.findtext('length', '0'))
                if is_static:
                    lines.append(f'    <geom name="{name}" type="cylinder" pos="{position}" euler="{rotation}" size="{radius:.6f} {length/2:.6f}"{common}/>')
                else:
                    # Mocap bodies let the isolated viewer mirror the same
                    # MarkerArray positions used by RViz and the lidar model.
                    lines.append(
                        f'    <body name="m20_dynamic_obstacle_{dynamic_count}" '
                        f'mocap="true" pos="{position}" euler="{rotation}">')
                    lines.append(
                        f'      <geom name="{name}" type="cylinder" pos="0 0 0" '
                        f'size="{radius:.6f} {length/2:.6f}"{common}/>')
                    lines.append('    </body>')
                    dynamic_count += 1
                count += 1
    lines += ['  </worldbody>', '</mujoco>', '']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(lines), encoding='utf-8')
    print(f'generated {count} geoms -> {args.output}')

if __name__ == '__main__':
    main()
