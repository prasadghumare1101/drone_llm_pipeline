"""Terrain clearance: is every leg of a step far enough above the GROUND?

Every altitude in a mission is metres above HOME, but the ground is not flat.
Over a hill that rises 16 m above home, a "15 m" leg is underground - which is
exactly how a drone flies into a hillside (it happened in the McMillan world).
This module checks the compiled path against a digital elevation model:

  * TerrainModel  - ground height above home at any (x, y), from a DEM GeoTIFF
                    placed in the local ENU frame (x = East, y = North, m)
  * check_clearance - samples every leg every SAMPLE_M metres (not just the
                    waypoints: a straight leg between two safe points can still
                    cross a ridge) plus the straight-line return home, and
                    reports the lowest clearance and the altitude that is safe.

Sim: the Gazebo world's own DEM (measured against Gazebo's ground to ~0.6 m).
Real world: an SRTM / survey DEM of the site, placed with the same two numbers.
Without a model the ground is assumed flat at home's height (as before).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from executor import ExecutableCommand, Goto, Land, ReturnToLaunch, Takeoff

SAMPLE_M = 5.0


class TerrainModel:
    """Ground height relative to home, bilinear-interpolated from a DEM grid.

    The DEM image (north up) is centred on home and spans size_m = (east-west,
    north-south) metres. Outside the DEM the nearest edge value is used.
    """

    def __init__(self, dem_path: str, size_m: Tuple[float, float]):
        import numpy as np                     # only needed when terrain is used
        from PIL import Image
        self._np = np
        self.grid = np.asarray(Image.open(dem_path), dtype=float)
        self.rows, self.cols = self.grid.shape
        self.size_x, self.size_y = float(size_m[0]), float(size_m[1])
        self._home = self._raw(0.0, 0.0)

    def _raw(self, x: float, y: float) -> float:
        c = (x + self.size_x / 2) / self.size_x * (self.cols - 1)
        r = (self.size_y / 2 - y) / self.size_y * (self.rows - 1)
        c = min(max(c, 0.0), self.cols - 1.0)
        r = min(max(r, 0.0), self.rows - 1.0)
        r0, c0 = int(r), int(c)
        r1, c1 = min(r0 + 1, self.rows - 1), min(c0 + 1, self.cols - 1)
        fr, fc = r - r0, c - c0
        g = self.grid
        return float(g[r0, c0] * (1 - fr) * (1 - fc) + g[r0, c1] * (1 - fr) * fc
                     + g[r1, c0] * fr * (1 - fc) + g[r1, c1] * fr * fc)

    def height(self, x: float, y: float) -> float:
        """Ground height above HOME (negative = below home) at (x, y)."""
        return self._raw(x, y) - self._home

    def max_height_within(self, x: float, y: float, radius: float, step: float = 50.0) -> float:
        best = self.height(x, y)
        n = max(1, int(radius / step))
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                if i * i + j * j <= n * n:
                    best = max(best, self.height(x + i * step, y + j * step))
        return best


@dataclass(frozen=True)
class ClearanceVerdict:
    ok: bool
    min_clearance_m: float          # lowest height above ground anywhere on the path
    worst_xy: Tuple[float, float]   # where that is
    ground_m: float                 # ground height (above home) there
    safe_alt_m: float               # lowest single altitude that clears the whole path
    message: str


def _legs(commands: Sequence[ExecutableCommand], start: Tuple[float, float, float],
          rtl_alt_m: float) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float], bool]]:
    """(from, to, is_cruise_leg) for every movement, in order."""
    x, y, z = start
    out = []
    for c in commands:
        if isinstance(c, Takeoff):
            x, y, z = x, y, c.alt                 # vertical climb from the ground: no sweep
        elif isinstance(c, Goto):
            out.append(((x, y, z), (c.x, c.y, c.alt), True))
            x, y, z = c.x, c.y, c.alt
        elif isinstance(c, ReturnToLaunch):       # PX4 RTL: climb to rtl_alt, fly home level
            z = max(z, rtl_alt_m)
            out.append(((x, y, z), (0.0, 0.0, z), True))
            return out
        elif isinstance(c, Land):
            return out
    return out


def check_clearance(commands: Sequence[ExecutableCommand], start: Tuple[float, float, float],
                    terrain: Optional[TerrainModel], min_clearance_m: float = 10.0,
                    rtl_alt_m: float = 30.0, include_return: bool = True) -> ClearanceVerdict:
    """Sample every leg (and, unless the step lands, the straight return home at
    PX4's RTL altitude) against the ground. ok = never closer than min_clearance_m."""
    if terrain is None:
        return ClearanceVerdict(True, float("inf"), (0.0, 0.0), 0.0, 0.0, "terrain not modelled")
    legs = _legs(commands, start, rtl_alt_m)
    ends_down = any(isinstance(c, (Land, ReturnToLaunch)) for c in commands)
    if include_return and not ends_down:
        end = legs[-1][1] if legs else start
        z = max(end[2], rtl_alt_m)
        legs.append(((end[0], end[1], z), (0.0, 0.0, z), False))
    worst = (float("inf"), (start[0], start[1]), 0.0, True)
    need = 0.0                                  # altitude that clears every cruise leg
    for (x0, y0, z0), (x1, y1, z1), cruise in legs:
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / SAMPLE_M))
        for i in range(n + 1):
            f = i / n
            x, y, z = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, z0 + (z1 - z0) * f
            g = terrain.height(x, y)
            if z - g < worst[0]:
                worst = (z - g, (x, y), g, cruise)
            need = max(need, g + min_clearance_m)
    clear, (wx, wy), g, cruise = worst
    if clear >= min_clearance_m:
        return ClearanceVerdict(True, clear, (wx, wy), g, need,
                                f"terrain ok: at least {clear:.0f} m above the ground")
    where = "on the way home afterwards" if not cruise else f"at ({wx:.0f}, {wy:.0f})"
    depth = (f"{-clear:.1f} m BELOW the ground" if clear < 0
             else f"only {clear:.1f} m above the ground")
    return ClearanceVerdict(
        False, clear, (wx, wy), g, need,
        f"the path passes {depth} {where} (ground there is {g:+.0f} m relative to home); "
        f"it needs {min_clearance_m:g} m clearance - fly it at {math.ceil(need):g} m or more")
