#!/usr/bin/env python3
# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Aggregate JSON evidence from independent boundary-recovery cold starts."""

import argparse
import json
from pathlib import Path

from m20_warehouse_inspection.boundary_recovery_probe import aggregate_reports


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'roots',
        nargs='+',
        type=Path,
        help='One or more evidence roots to combine',
    )
    parser.add_argument(
        '--output', type=Path, default=None
    )
    args = parser.parse_args()
    paths = sorted(
        {
            path
            for root in args.roots
            for path in root.glob('**/boundary_recovery_summary.json')
        }
    )
    if not paths:
        raise RuntimeError(
            f'no boundary recovery reports below {args.roots}'
        )
    reports = [
        json.loads(path.read_text(encoding='utf-8')) for path in paths
    ]
    summary = aggregate_reports(reports)
    summary['source_reports'] = [str(path) for path in paths]
    output = (
        args.output
        or args.roots[0] / 'boundary_recovery_matrix_summary.json'
    )
    output.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary['all_runs_accepted'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
