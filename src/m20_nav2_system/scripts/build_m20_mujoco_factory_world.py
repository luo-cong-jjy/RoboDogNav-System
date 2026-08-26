#!/usr/bin/env python3
"""Inject generated static factory geoms into the official M20 MJCF template."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('template', type=Path)
    parser.add_argument('geoms', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--meshdir', default='', help='MJCF mesh directory')
    args = parser.parse_args()
    template = args.template.read_text(encoding='utf-8')
    fragment = args.geoms.read_text(encoding='utf-8')
    start = fragment.find('<worldbody>')
    end = fragment.rfind('</worldbody>')
    if start < 0 or end < 0:
        raise SystemExit('geoms file does not contain a worldbody')
    body = fragment[start + len('<worldbody>'):end]
    marker = '        <!-- ==================== base_link：机身'
    if marker not in template:
        marker = '    <body name="base_link"'
    if marker not in template:
        raise SystemExit('M20 template base_link marker not found')
    output = template.replace(marker, '        <!-- generated factory static geoms -->\n' + body + '\n' + marker, 1)
    if args.meshdir:
        import re
        output = re.sub(r'<compiler angle="radian" meshdir="[^"]*"/>',
                        f'<compiler angle="radian" meshdir="{Path(args.meshdir).resolve()}"/>',
                        output, count=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding='utf-8')
    print(f'generated factory MJCF -> {args.output}')


if __name__ == '__main__':
    main()
