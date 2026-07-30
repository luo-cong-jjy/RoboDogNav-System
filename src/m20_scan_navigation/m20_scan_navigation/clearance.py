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

"""Straight-passage clearance calculations for the M20 navigation stack."""

from dataclasses import dataclass
from pathlib import Path
import math
from typing import Iterable, List, Mapping, Optional, Tuple

import yaml


M20_BODY_LENGTH_M = 0.82
M20_BODY_WIDTH_M = 0.51


@dataclass(frozen=True)
class ClearanceProfile:
    """Parameters shared by SCAN, optional grid A*, and collision guard."""

    name: str
    body_radius: float
    optimizer_clearance: float
    guard_footprint_radius: float
    guard_safety_margin: float
    grid_route_inflation: float

    @property
    def physical_minimum_width(self) -> float:
        """Body-only geometric limit for a perfectly aligned straight pass."""
        return M20_BODY_WIDTH_M

    @property
    def scan_hard_width(self) -> float:
        """Width represented by SCAN's double-cylinder cross section."""
        return 2.0 * self.body_radius

    @property
    def guard_hard_width(self) -> float:
        """Width below which the independent static-map guard must stop."""
        return 2.0 * (
            self.guard_footprint_radius + self.guard_safety_margin
        )

    @property
    def scan_preferred_width(self) -> float:
        """Nominal width requested by SCAN's obstacle cost."""
        return 2.0 * (
            self.body_radius + self.optimizer_clearance
        )

    @property
    def grid_route_width(self) -> float:
        """Nominal corridor width required by optional grid A*."""
        return 2.0 * self.grid_route_inflation

    @property
    def native_hard_width(self) -> float:
        """Hard lower bound for the normal native-SCAN system."""
        return max(
            self.physical_minimum_width,
            self.scan_hard_width,
            self.guard_hard_width,
        )

    @property
    def native_nominal_width(self) -> float:
        """Planner target before a passage is considered nominal."""
        return max(self.native_hard_width, self.scan_preferred_width)

    def raster_guard_exclusive_width(self, resolution: float) -> float:
        """
        Return the conservative occupied-grid boundary for the guard.

        The current guard expands whole cells and then samples the cell
        containing the body centre. One additional cell per wall captures
        obstacle rasterization and centre-cell quantization. A doorway must
        be strictly wider than this value; the controlled maps determine the
        first representable passing width.
        """
        cell_size = float(resolution)
        if cell_size <= 0.0:
            raise ValueError('map resolution must be positive')
        guard_radius = (
            self.guard_footprint_radius + self.guard_safety_margin
        )
        kernel_cells = int(math.ceil(guard_radius / cell_size))
        return 2.0 * (kernel_cells + 1) * cell_size

    def classify(
        self,
        passage_width: float,
        use_grid_route: bool = False,
        map_resolution: Optional[float] = None,
    ) -> str:
        """Classify one nominal straight width without claiming dynamics."""
        width = float(passage_width)
        if width + 1e-9 < self.physical_minimum_width:
            return 'PHYSICALLY_BLOCKED'
        if (
            map_resolution is not None
            and width
            <= self.raster_guard_exclusive_width(map_resolution) + 1e-9
        ):
            return 'GRID_GUARD_BLOCKED'
        if width + 1e-9 < self.native_hard_width:
            return 'GUARD_BLOCKED'
        route_minimum = (
            max(self.native_hard_width, self.grid_route_width)
            if use_grid_route
            else self.native_hard_width
        )
        if width + 1e-9 < route_minimum:
            return 'ROUTE_BLOCKED'
        if width + 1e-9 < self.native_nominal_width:
            return 'MARGINAL'
        return 'NOMINAL_CANDIDATE'


def load_clearance_profile(path: Path) -> ClearanceProfile:
    """Load one explicit ROS parameter override and validate its fields."""
    profile_path = Path(path).expanduser().resolve()
    with profile_path.open('r', encoding='utf-8') as stream:
        root = yaml.safe_load(stream)
    if not isinstance(root, Mapping):
        raise ValueError(f'{profile_path} must contain a YAML mapping')

    def parameters(node_name: str) -> Mapping:
        node = root.get(node_name)
        if not isinstance(node, Mapping):
            raise ValueError(f'{profile_path} has no {node_name} section')
        values = node.get('ros__parameters')
        if not isinstance(values, Mapping):
            raise ValueError(
                f'{profile_path} has no {node_name}.ros__parameters'
            )
        return values

    planner = parameters('scan_planner_node')
    route = parameters('m20_grid_route_planner')
    guard = parameters('m20_collision_guard')
    numeric_fields = {
        'body_radius': planner.get('grid_map.double_cylinder_radius'),
        'optimizer_clearance': planner.get('optimization.dist0'),
        'guard_footprint_radius': guard.get('footprint_radius'),
        'guard_safety_margin': guard.get('safety_margin'),
        'grid_route_inflation': route.get('inflation_radius'),
    }
    for key, value in numeric_fields.items():
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or float(value) < 0.0
        ):
            raise ValueError(
                f'{profile_path}: {key} must be a non-negative number'
            )
    return ClearanceProfile(
        name=profile_path.stem.removeprefix('clearance_'),
        **{key: float(value) for key, value in numeric_fields.items()},
    )


def passage_table(
    profile: ClearanceProfile,
    widths: Iterable[float],
    use_grid_route: bool = False,
    map_resolution: Optional[float] = None,
) -> List[Tuple[float, str]]:
    """Return stable width/status rows for reports and regression tests."""
    return [
        (
            float(width),
            profile.classify(
                width,
                use_grid_route,
                map_resolution=map_resolution,
            ),
        )
        for width in widths
    ]
