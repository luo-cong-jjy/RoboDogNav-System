#!/usr/bin/env python3
"""Rasterize static SDF collision geometry into a Nav2 occupancy map."""
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def vector(text, count):
    values = [float(value) for value in (text or '').split()]
    return (values + [0.0] * count)[:count]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('world', type=Path)
    parser.add_argument('yaml', type=Path)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--padding', type=float, default=0.5)
    args = parser.parse_args()
    root = ET.parse(args.world).getroot()
    shapes, ground_bounds = [], None
    for model in root.findall('.//world/model'):
        if model.findtext('static', 'false').strip().lower() != 'true':
            continue
        name = model.attrib.get('name', '')
        mx, my, mz, _, _, myaw = vector(model.findtext('pose'), 6)
        for collision in model.findall('.//collision'):
            geometry = collision.find('geometry')
            if geometry is None:
                continue
            cx, cy, cz, _, _, cyaw = vector(collision.findtext('pose'), 6)
            x, y, z, yaw = mx + cx, my + cy, mz + cz, myaw + cyaw
            box = geometry.find('box/size')
            cylinder = geometry.find('cylinder')
            if box is not None:
                sx, sy, sz = vector(box.text, 3)
                if name == 'ground':
                    ground_bounds = (x - sx / 2, x + sx / 2,
                                     y - sy / 2, y + sy / 2)
                elif z + sz / 2 > 0.05:
                    shapes.append(('box', x, y, sx, sy, yaw))
            elif cylinder is not None:
                radius = float(cylinder.findtext('radius', '0'))
                length = float(cylinder.findtext('length', '0'))
                if z + length / 2 > 0.05:
                    shapes.append(('cylinder', x, y, radius, radius, 0.0))
    if ground_bounds is None:
        raise SystemExit('static ground box is required to define map bounds')
    min_x, max_x, min_y, max_y = ground_bounds
    min_x, max_x = min_x - args.padding, max_x + args.padding
    min_y, max_y = min_y - args.padding, max_y + args.padding
    resolution = args.resolution
    width = math.ceil((max_x - min_x) / resolution)
    height = math.ceil((max_y - min_y) / resolution)
    pixels = bytearray([254]) * (width * height)
    for row in range(height):
        y = min_y + (row + 0.5) * resolution
        image_row = height - 1 - row
        for column in range(width):
            x = min_x + (column + 0.5) * resolution
            for kind, sx, sy, extent_x, extent_y, yaw in shapes:
                dx, dy = x - sx, y - sy
                if kind == 'cylinder':
                    occupied = dx * dx + dy * dy <= extent_x * extent_x
                else:
                    cosine, sine = math.cos(yaw), math.sin(yaw)
                    local_x = cosine * dx + sine * dy
                    local_y = -sine * dx + cosine * dy
                    occupied = (abs(local_x) <= extent_x / 2 and
                                abs(local_y) <= extent_y / 2)
                if occupied:
                    pixels[image_row * width + column] = 0
                    break
    pgm_path = args.yaml.with_suffix('.pgm')
    args.yaml.parent.mkdir(parents=True, exist_ok=True)
    with pgm_path.open('wb') as stream:
        stream.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        stream.write(pixels)
    args.yaml.write_text(
        f'image: {pgm_path.name}\nmode: trinary\nresolution: {resolution}\n'
        f'origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
    print(f'generated {width}x{height} Nav2 map from {len(shapes)} static geoms')


if __name__ == '__main__':
    main()
