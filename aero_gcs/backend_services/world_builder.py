#!/usr/bin/env python3
"""Build a Gazebo-classic world of a REAL location: elevation + aerial image.

    python3 world_builder.py <name> <lat> <lon> [--size 2000]

Produces, for a square of `size` metres centred on (lat, lon) = home:
  ~/.gazebo/models/<name>/          heightmap.png (257x257, 8-bit) + texture.jpg
                                    (2048 px) + dem.tif (float heights, same grid)
  aero_gcs/sim_worlds/<name>.world  PX4's empty-world physics + this terrain +
                                    the location's spherical coordinates
  aero_gcs/backend_services/basemaps.json entry: aerial map placement, terrain
                                    model for the clearance check, geofence, location

Kept deliberately light so a laptop GPU renders it smoothly (McMillan's texture
alone is 63 MB; this whole world is a few MB). A small pad around home is
flattened so the vehicle spawns level, like a real take-off spot.

Sources (no keys): elevation = AWS Terrain Tiles ("terrarium", SRTM-based,
global incl. India, ~9 m at z14); imagery = Esri World Imagery (~1 m/px at z17,
attribution required: "Esri, Maxar, Earthstar Geographics"; check Esri's terms
for commercial use - your own drone orthophoto is the best source).
"""
from __future__ import annotations

import json
import math
import re
import time
import urllib.request
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from PIL import Image

SERVICES_DIR = Path(__file__).resolve().parent
TILE_CACHE = SERVICES_DIR / "cache" / "tiles"
WORLDS_OUT = SERVICES_DIR.parent / "sim_worlds"
MODELS_OUT = Path.home() / ".gazebo" / "models"
# Generated sites are machine-specific (their worlds/models live outside the
# repo) and record where the operator flies, so they go to a git-ignored file
# next to the committed basemaps.json; control_api reads both.
BASEMAPS = SERVICES_DIR / "cache" / "sites.json"

ELEV_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
ELEV_Z = 14
IMG_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
IMG_Z = 17
IMG_ATTRIBUTION = "Imagery: Esri, Maxar, Earthstar Geographics; elevation: AWS Terrain Tiles (SRTM)"
HEIGHTMAP_N = 257            # Gazebo needs 2^n + 1
TEXTURE_PX = 2048
PAD_R, PAD_BLEND = 15.0, 40.0    # flat launch pad radius, blend-out radius (m)
MAX_TILES = 400
R_E = 6378137.0
UA = {"user-agent": "drone-llm-pipeline-world-builder/1.0"}


def _tile_frac(lat: float, lon: float, z: int) -> tuple:
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def _ll(lat0: float, lon0: float, east: float, north: float) -> tuple:
    return (lat0 + math.degrees(north / R_E),
            lon0 + math.degrees(east / (R_E * math.cos(math.radians(lat0)))))


def _fetch(url: str, dest: Path) -> Path:
    if dest.is_file() and dest.stat().st_size > 0:
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    for attempt in range(3):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=20) as r:
                dest.write_bytes(r.read())
            return dest
        except Exception:
            if attempt == 2:
                raise
            time.sleep(1.0 + attempt)
    return dest


class _Mosaic:
    """Tiles covering a lat/lon box, stitched; sampled by lat/lon."""

    def __init__(self, url: str, kind: str, z: int, lat0: float, lon0: float, half_m: float,
                 progress: Callable[[str], None]):
        nw = _ll(lat0, lon0, -half_m * 1.1, half_m * 1.1)
        se = _ll(lat0, lon0, half_m * 1.1, -half_m * 1.1)
        x0, y0 = (int(v) for v in _tile_frac(*nw, z))
        x1, y1 = (int(v) for v in _tile_frac(*se, z))
        count = (x1 - x0 + 1) * (y1 - y0 + 1)
        if count > MAX_TILES:
            raise ValueError(f"{kind}: {count} tiles needed - choose a smaller area")
        self.z, self.x0, self.y0 = z, x0, y0
        self.img = Image.new("RGB", ((x1 - x0 + 1) * 256, (y1 - y0 + 1) * 256))
        ext = "png" if "terrarium" in url else "jpg"
        done = 0
        for ty in range(y0, y1 + 1):
            for tx in range(x0, x1 + 1):
                f = _fetch(url.format(z=z, x=tx, y=ty), TILE_CACHE / kind / str(z) / str(tx) / f"{ty}.{ext}")
                with Image.open(f) as t:
                    self.img.paste(t.convert("RGB"), ((tx - x0) * 256, (ty - y0) * 256))
                done += 1
                if done % 10 == 0 or done == count:
                    progress(f"{kind}: {done}/{count} tiles")

    def px(self, lat: float, lon: float) -> tuple:
        x, y = _tile_frac(lat, lon, self.z)
        return (x - self.x0) * 256.0, (y - self.y0) * 256.0


def _bilinear(a: np.ndarray, u: float, v: float) -> float:
    h, w = a.shape
    u = min(max(u, 0.0), w - 1.001)
    v = min(max(v, 0.0), h - 1.001)
    c0, r0 = int(u), int(v)
    fu, fv = u - c0, v - r0
    return float(a[r0, c0] * (1 - fu) * (1 - fv) + a[r0, c0 + 1] * fu * (1 - fv)
                 + a[r0 + 1, c0] * (1 - fu) * fv + a[r0 + 1, c0 + 1] * fu * fv)


def build(name: str, lat: float, lon: float, size_m: float = 2000.0,
          progress: Callable[[str], None] = print) -> dict:
    name = re.sub(r"[^a-z0-9_]", "_", name.lower()).strip("_")[:40] or "field"
    if not 500 <= size_m <= 3000:
        raise ValueError("size must be 500-3000 m")
    half = size_m / 2.0
    model_dir = MODELS_OUT / name
    media = model_dir / "media"
    media.mkdir(parents=True, exist_ok=True)

    # ---- elevation -> 257 x 257 grid, north-up, metres MSL ---------------- #
    progress("downloading elevation tiles…")
    em = _Mosaic(ELEV_URL, "terrarium", ELEV_Z, lat, lon, half, progress)
    rgb = np.asarray(em.img, dtype=np.float64)
    elev = rgb[:, :, 0] * 256.0 + rgb[:, :, 1] + rgb[:, :, 2] / 256.0 - 32768.0
    n = HEIGHTMAP_N
    grid = np.zeros((n, n))
    for i in range(n):                     # row 0 = north edge
        north = half - i * size_m / (n - 1)
        for j in range(n):                 # col 0 = west edge
            east = -half + j * size_m / (n - 1)
            u, v = em.px(*_ll(lat, lon, east, north))
            grid[i, j] = _bilinear(elev, u, v)

    # ---- flat launch pad at home ----------------------------------------- #
    c = (n - 1) / 2.0
    cell = size_m / (n - 1)
    h_home = float(np.median([grid[i, j] for i in range(n) for j in range(n)
                              if math.hypot(i - c, j - c) * cell <= PAD_R] or [grid[int(c), int(c)]]))
    for i in range(n):
        for j in range(n):
            d = math.hypot(i - c, j - c) * cell
            if d <= PAD_BLEND:
                k = 1.0 if d <= PAD_R else (PAD_BLEND - d) / (PAD_BLEND - PAD_R)
                grid[i, j] = k * h_home + (1 - k) * grid[i, j]

    # ---- 8-bit heightmap + the float DEM of exactly what Gazebo will use --- #
    hmin, hmax = float(grid.min()), float(grid.max())
    span = max(hmax - hmin, 1.0)
    q = np.round((grid - hmin) / span * 255.0)
    Image.fromarray(q.astype(np.uint8), mode="L").save(media / "heightmap.png")
    dem = (hmin + q / 255.0 * span).astype(np.float32)
    Image.fromarray(dem, mode="F").save(media / "dem.tif")
    pos_z = hmin - h_home                  # puts home's ground at z = 0

    # ---- aerial texture: 2048 px, north-up, exactly the terrain square ---- #
    progress("downloading aerial imagery tiles…")
    im = _Mosaic(IMG_URL, "esri_world_imagery", IMG_Z, lat, lon, half, progress)
    corners = [im.px(*_ll(lat, lon, e, nn)) for e, nn in
               ((-half, half), (-half, -half), (half, -half), (half, half))]   # NW SW SE NE
    tex = im.img.transform((TEXTURE_PX, TEXTURE_PX), Image.QUAD,
                           data=tuple(v for p in corners for v in p), resample=Image.BICUBIC)
    tex.save(media / "texture.jpg", quality=88)

    # ---- Gazebo model + world ------------------------------------------- #
    progress("writing Gazebo model and world…")
    hm = (f"<uri>model://{name}/media/heightmap.png</uri>\n          "
          f"<size>{size_m:g} {size_m:g} {span:.2f}</size>\n          <pos>0 0 {pos_z:.2f}</pos>")
    (model_dir / "model.config").write_text(
        f"<?xml version='1.0'?>\n<model><name>{name}</name><version>1.0</version>"
        f"<sdf version='1.5'>model.sdf</sdf><description>Real terrain at {lat:.5f}, {lon:.5f} "
        f"({size_m:g} m square). {IMG_ATTRIBUTION}</description></model>\n", encoding="utf-8")
    (model_dir / "model.sdf").write_text(f"""<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry><heightmap>
          {hm}
        </heightmap></geometry>
      </collision>
      <visual name='visual'>
        <geometry><heightmap>
          <use_terrain_paging>false</use_terrain_paging>
          <texture>
            <diffuse>model://{name}/media/texture.jpg</diffuse>
            <normal>file://media/materials/textures/flat_normal.png</normal>
            <size>{size_m:g}</size>
          </texture>
          {hm}
        </heightmap></geometry>
      </visual>
    </link>
  </model>
</sdf>
""", encoding="utf-8")
    WORLDS_OUT.mkdir(parents=True, exist_ok=True)
    world = WORLDS_OUT / f"{name}.world"
    world.write_text(f"""<?xml version='1.0' ?>
<sdf version='1.5'>
  <!-- generated by aero_gcs world_builder.py: {lat:.6f}, {lon:.6f}, {size_m:g} m.
       {IMG_ATTRIBUTION} -->
  <world name='default'>
    <scene>
      <!-- plain sky colour + no shadows: looks natural in the FPV feed, costs the GPU nothing -->
      <background>0.62 0.76 0.92 1</background>
      <ambient>0.5 0.5 0.5 1</ambient>
      <shadows>false</shadows>
    </scene>
    <include><uri>model://sun</uri></include>
    <include><uri>model://{name}</uri></include>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{lat:.7f}</latitude_deg>
      <longitude_deg>{lon:.7f}</longitude_deg>
      <elevation>{h_home:.1f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver><type>quick</type><iters>10</iters><sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling></solver>
        <constraints><cfm>0</cfm><erp>0.2</erp><contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer></constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
  </world>
</sdf>
""", encoding="utf-8")

    # ---- dashboard map + terrain check + geofence + location ------------- #
    cfg = json.loads(BASEMAPS.read_text(encoding="utf-8")) if BASEMAPS.is_file() else {}
    previous = cfg.get(name, {}) if cfg.get(name, {}).get("generated", {}).get("size_m") == size_m else {}
    cfg[name] = {
        "texture": f"{name}/media/texture.jpg",
        "affine": [[TEXTURE_PX / size_m, 0.0, TEXTURE_PX / 2.0], [0.0, -TEXTURE_PX / size_m, TEXTURE_PX / 2.0]],
        "texture_size_px": [TEXTURE_PX, TEXTURE_PX],
        "terrain": {"dem": f"{name}/media/dem.tif", "size_m": [size_m, size_m]},
        "geofence_half_side_m": max(200.0, half - 50.0),
        "min_terrain_clearance_m": 10,
        "rtl_alt_m": 30,
        "location": {"lat": lat, "lon": lon, "alt_msl": round(h_home, 1)},
        "generated": {"by": "world_builder.py", "size_m": size_m, "attribution": IMG_ATTRIBUTION,
                      "relief_m": round(span, 1), "ground_msl_m": [round(hmin, 1), round(hmax, 1)]},
    }
    # a rebuild must never quietly undo a safety setting someone tuned for the site
    for k in ("min_terrain_clearance_m", "rtl_alt_m", "geofence_half_side_m", "_note"):
        if k in previous:
            cfg[name][k] = previous[k]
    BASEMAPS.parent.mkdir(parents=True, exist_ok=True)
    BASEMAPS.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
    summary = {"name": name, "world": str(world), "model": str(model_dir), "size_m": size_m,
               "home_msl_m": round(h_home, 1), "relief_m": round(span, 1),
               "ground_range_rel_home_m": [round(hmin - h_home, 1), round(hmax - h_home, 1)],
               "geofence_half_side_m": cfg[name]["geofence_half_side_m"]}
    progress(f"done: {name}")
    return summary


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("name")
    ap.add_argument("lat", type=float)
    ap.add_argument("lon", type=float)
    ap.add_argument("--size", type=float, default=2000.0)
    a = ap.parse_args()
    print(json.dumps(build(a.name, a.lat, a.lon, a.size), indent=2))
