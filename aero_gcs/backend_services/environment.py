"""Simulated environment for a real location: weather + geodetic placement.

What a real place contributes to the PX4 / Gazebo-classic sim:
  * PX4_HOME_LAT / LON / ALT  -> GPS reads that location; PX4's world magnetic
    model then gives that place's compass field, and the barometer that
    elevation's air pressure (Gazebo GPS / baro / ground-truth plugins read them)
  * wind                      -> a copy of the selected world with Gazebo's wind
    plugin injected: mean wind, turbulence (random variation) and a gust window.
    The iris motor model feels it, so the drone is really pushed around.
The 3D terrain stays the selected Gazebo world; only the places above change.

Weather comes from Open-Meteo (free, no key; only latitude/longitude are sent).
"""
from __future__ import annotations

import json
import math
import re
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Optional

OPEN_METEO = "https://api.open-meteo.com/v1/forecast"
SHEAR_EXPONENT = 0.14          # wind power law, open terrain: v(h) = v10 * (h/10)^a
REFERENCE_FLIGHT_ALT_M = 30.0  # wind is set for this height above ground
GUST_START_S = 60.0            # first gust window, sim seconds
GUST_DURATION_S = 20.0


def fetch_weather(lat: float, lon: float, timeout: float = 15.0) -> dict:
    """Current conditions at (lat, lon): wind at 10 m (m/s, direction it blows
    FROM, degrees true), gusts, temperature, surface pressure, ground elevation."""
    q = urllib.parse.urlencode({
        "latitude": f"{lat:.5f}", "longitude": f"{lon:.5f}", "wind_speed_unit": "ms",
        "current": "temperature_2m,surface_pressure,wind_speed_10m,wind_direction_10m,wind_gusts_10m",
    })
    req = urllib.request.Request(f"{OPEN_METEO}?{q}", headers={"user-agent": "drone-llm-pipeline/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        d = json.loads(r.read().decode("utf-8"))
    c = d["current"]
    return {"time": c["time"], "wind_mps": c["wind_speed_10m"], "wind_from_deg": c["wind_direction_10m"],
            "gust_mps": c["wind_gusts_10m"], "temp_c": c["temperature_2m"],
            "pressure_hpa": c["surface_pressure"], "elevation_m": d.get("elevation"),
            "source": "open-meteo"}


def wind_at(v10: float, height_m: float = REFERENCE_FLIGHT_ALT_M) -> float:
    return v10 * (max(height_m, 1.0) / 10.0) ** SHEAR_EXPONENT


def _to_vector(from_deg: float) -> str:
    """Meteorological 'from' bearing -> Gazebo unit vector it blows TO (x=E, y=N)."""
    to = math.radians(from_deg + 180.0)
    return f"{math.sin(to):.4f} {math.cos(to):.4f} 0"


def wind_plugin_xml(env: dict) -> str:
    mean = wind_at(float(env.get("wind_mps", 0.0)))
    gust = max(0.0, wind_at(float(env.get("gust_mps", 0.0))) - mean)
    turb = float(env.get("turbulence_mps", round(0.25 * gust, 2)))
    vec = _to_vector(float(env.get("wind_from_deg", 0.0)))
    return f"""
    <!-- injected by aero_gcs environment.py: {env.get('name', 'location')} -->
    <plugin name='aero_env_wind' filename='libgazebo_wind_plugin.so'>
      <frameId>base_link</frameId>
      <robotNamespace/>
      <windVelocityMean>{mean:.2f}</windVelocityMean>
      <windVelocityMax>{max(20.0, mean + 3 * turb):.1f}</windVelocityMax>
      <windVelocityVariance>{turb ** 2:.3f}</windVelocityVariance>
      <windDirectionMean>{vec}</windDirectionMean>
      <windDirectionVariance>0.01</windDirectionVariance>
      <windGustStart>{GUST_START_S:g}</windGustStart>
      <windGustDuration>{GUST_DURATION_S if gust > 0 else 0:g}</windGustDuration>
      <windGustVelocityMean>{gust:.2f}</windGustVelocityMean>
      <windGustVelocityMax>{max(20.0, gust * 2):.1f}</windGustVelocityMax>
      <windGustVelocityVariance>{(0.3 * gust) ** 2:.3f}</windGustVelocityVariance>
      <windGustDirectionMean>{vec}</windGustDirectionMean>
      <windGustDirectionVariance>0.02</windGustDirectionVariance>
    </plugin>
"""


def build_world(src: Path, dst: Path, env: dict) -> Path:
    """Copy `src` world with the location's spherical coordinates and the wind
    plugin injected (any wind plugin already in the world is removed first)."""
    xml = src.read_text(encoding="utf-8")
    xml = re.sub(r"\s*<plugin[^>]*libgazebo_wind_plugin\.so.*?</plugin>", "", xml, flags=re.S)
    sph = (f"<spherical_coordinates>\n      <surface_model>EARTH_WGS84</surface_model>\n"
           f"      <latitude_deg>{float(env['lat']):.7f}</latitude_deg>\n"
           f"      <longitude_deg>{float(env['lon']):.7f}</longitude_deg>\n"
           f"      <elevation>{float(env.get('alt_msl', 0.0)):.1f}</elevation>\n"
           f"      <heading_deg>0</heading_deg>\n    </spherical_coordinates>")
    if "<spherical_coordinates>" in xml:
        xml = re.sub(r"<spherical_coordinates>.*?</spherical_coordinates>", sph, xml, count=1, flags=re.S)
    else:
        xml = xml.replace("</world>", f"    {sph}\n  </world>", 1)
    xml = xml.replace("</world>", wind_plugin_xml(env) + "  </world>", 1)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(xml, encoding="utf-8")
    return dst


def summary(env: Optional[dict]) -> str:
    if not env or not env.get("enabled"):
        return "world default (no location / weather)"
    return (f"{env.get('name') or 'location'} ({float(env['lat']):.4f}, {float(env['lon']):.4f}, "
            f"{float(env.get('alt_msl', 0)):.0f} m) - wind {float(env.get('wind_mps', 0)):.1f} m/s "
            f"from {float(env.get('wind_from_deg', 0)):.0f} deg, gusts {float(env.get('gust_mps', 0)):.1f} m/s")
