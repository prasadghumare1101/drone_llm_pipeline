#!/usr/bin/env python3
"""Real-flight log analyser: ArduPilot dataflash (.BIN) and MAVLink telemetry (.tlog).

Builds a timeline (modes, arming, mission progress, autopilot messages) and
automatic FINDINGS - above all "stuck" episodes: the vehicle sat on one mission
item, barely moving, in an autonomous mode. For each it names the likely reason
(waiting for a yaw, waypoint never reached, a delay/loiter item, mission ended
without RTL/LAND, EKF / GPS trouble) and what to change.

Also: failsafe / crash / EKF / GPS / vibration messages, low battery, telemetry
gaps and the 3DR radio's link quality (RADIO_STATUS: RSSI, noise, rx errors).

Pure analysis - reads a file, returns a dict. No vehicle I/O.
    python3 flight_log_analyzer.py <log> [--json]
"""
from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional

from pymavlink import mavutil

STUCK_MIN_S = 20.0          # same mission item, autonomous mode, for at least this long...
STUCK_SPEED_MPS = 0.7       # ...while slower than this
WP_RADIUS_DEFAULT_M = 2.0   # ArduPilot WPNAV_RADIUS default
NAV_NAMES = {16: "WAYPOINT", 17: "LOITER_UNLIM", 18: "LOITER_TURNS", 19: "LOITER_TIME",
             20: "RTL", 21: "LAND", 22: "TAKEOFF", 93: "DELAY", 115: "CONDITION_YAW",
             177: "DO_JUMP", 178: "DO_CHANGE_SPEED", 201: "DO_SET_ROI", 112: "CONDITION_DELAY",
             114: "CONDITION_DISTANCE", 206: "DO_SET_CAM_TRIGG_DIST", 183: "DO_SET_SERVO"}
NAV_CMDS = {16, 17, 18, 19, 20, 21, 22, 93}
AUTO_MODES = {"AUTO", "GUIDED", "MISSION", "AUTO.MISSION", "OFFBOARD"}
ALERT = re.compile(r"failsafe|fail|ekf|gps glitch|glitch|crash|thrust loss|vibration|compass|"
                   r"battery|low|prearm|error|lost|fence|terrain|yaw imbalance|radio|"
                   r"not healthy|unhealthy|emergency|land now", re.I)
R_E = 6378137.0


def _ne(home, lat, lon):
    return (math.radians(lat - home[0]) * R_E,
            math.radians(lon - home[1]) * R_E * math.cos(math.radians(home[0])))


class _Acc:
    """Everything the parsers collect, in one normalised shape."""

    def __init__(self):
        self.t0: Optional[float] = None
        self.t_last = 0.0
        self.autopilot = "unknown"
        self.timeline: List[dict] = []
        self.mode: Optional[str] = None
        self.armed: Optional[bool] = None
        self.seq_changes: List[tuple] = []      # (t, seq)
        self.items: Dict[int, dict] = {}        # seq -> {cmd, p1..p4, lat, lon, alt}
        self.pos: List[tuple] = []              # (t, lat, lon, rel_alt, speed)
        self.home: Optional[tuple] = None
        self.batt: List[tuple] = []             # (t, pct, volt)
        self.gps: List[tuple] = []              # (t, fix, sats, hdop)
        self.radio: List[tuple] = []            # (t, rssi, remrssi, noise, remnoise, rxerrors)
        self.hb_times: List[float] = []
        self.ekf_bad: List[tuple] = []          # (t, text)
        self.mode_spans: List[tuple] = []       # (t_start, mode, armed)
        self.params: Dict[str, float] = {}

    def t(self, raw: float) -> float:
        if self.t0 is None:
            self.t0 = raw
        self.t_last = max(self.t_last, raw - self.t0)
        return raw - self.t0

    def event(self, t, kind, text, sev="info"):
        if self.timeline and self.timeline[-1]["text"] == text and t - self.timeline[-1]["t"] < 2:
            return                                  # duplicated over two links
        self.timeline.append({"t": round(t, 1), "kind": kind, "text": text, "severity": sev})

    def set_mode(self, t, mode, armed):
        if (mode, armed) != (self.mode, self.armed):
            if armed != self.armed and self.armed is not None:
                self.event(t, "arm", "ARMED" if armed else "DISARMED")
            if mode != self.mode:
                self.event(t, "mode", f"mode {mode}")
            self.mode, self.armed = mode, armed
            self.mode_spans.append((t, mode, armed))


# --------------------------------------------------------------------------- #
# parsers
# --------------------------------------------------------------------------- #

def _parse_tlog(path: str, acc: _Acc) -> None:
    m = mavutil.mavlink_connection(path, dialect="ardupilotmega", robust_parsing=True)
    fc_sys = None
    types = ["HEARTBEAT", "STATUSTEXT", "MISSION_CURRENT", "MISSION_ITEM_INT", "MISSION_ITEM",
             "GLOBAL_POSITION_INT", "HOME_POSITION", "SYS_STATUS", "GPS_RAW_INT", "RADIO_STATUS",
             "EKF_STATUS_REPORT", "PARAM_VALUE"]
    while True:
        msg = m.recv_match(type=types, blocking=False)
        if msg is None:
            break
        k = msg.get_type()
        if k == "BAD_DATA":
            continue
        t = acc.t(getattr(msg, "_timestamp", 0.0))
        src = msg.get_srcSystem()
        if k == "RADIO_STATUS":                  # from the 3DR radio itself (sysid 51 / 'R')
            acc.radio.append((t, msg.rssi, msg.remrssi, msg.noise, msg.remnoise, msg.rxerrors))
            continue
        if k == "HEARTBEAT":
            if msg.autopilot in (mavutil.mavlink.MAV_AUTOPILOT_INVALID,) or msg.type in (6, 18):
                continue                         # GCS / onboard computer heartbeats
            fc_sys = fc_sys or src
            if src != fc_sys:
                continue
            acc.autopilot = {3: "ArduPilot", 12: "PX4"}.get(msg.autopilot, str(msg.autopilot))
            acc.hb_times.append(t)
            acc.set_mode(t, mavutil.mode_string_v10(msg), bool(msg.base_mode & 128))
            continue
        if fc_sys is not None and src != fc_sys and k not in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            continue
        if k == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
            acc.event(t, "msg", text.strip(), "warning" if msg.severity <= 4 or ALERT.search(text) else "info")
        elif k == "MISSION_CURRENT":
            if not acc.seq_changes or acc.seq_changes[-1][1] != msg.seq:
                acc.seq_changes.append((t, msg.seq))
        elif k in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            scale = 1e7 if k == "MISSION_ITEM_INT" else 1.0
            acc.items[msg.seq] = {"cmd": msg.command, "p": [msg.param1, msg.param2, msg.param3, msg.param4],
                                  "lat": msg.x / scale, "lon": msg.y / scale, "alt": msg.z}
        elif k == "GLOBAL_POSITION_INT" and msg.lat:
            acc.pos.append((t, msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0,
                            math.hypot(msg.vx, msg.vy) / 100.0))
        elif k == "HOME_POSITION":
            acc.home = (msg.latitude / 1e7, msg.longitude / 1e7)
        elif k == "SYS_STATUS" and msg.battery_remaining >= 0:
            acc.batt.append((t, msg.battery_remaining, msg.voltage_battery / 1000.0))
        elif k == "GPS_RAW_INT":
            acc.gps.append((t, msg.fix_type, msg.satellites_visible, msg.eph / 100.0))
        elif k == "EKF_STATUS_REPORT":
            if not (msg.flags & 0x08 and msg.flags & 0x10) and acc.armed:   # horiz pos rel/abs
                acc.ekf_bad.append((t, f"EKF horizontal position not OK (flags 0x{msg.flags:x})"))
        elif k == "PARAM_VALUE":
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            acc.params[pid.strip("\x00")] = msg.param_value


def _parse_bin(path: str, acc: _Acc) -> None:
    m = mavutil.mavlink_connection(path)
    acc.autopilot = "ArduPilot"
    types = ["MODE", "MSG", "CMD", "POS", "GPS", "BAT", "ARM", "ERR", "EV", "PARM", "ORGN", "CTUN", "XKF4"]
    armed = False
    while True:
        msg = m.recv_match(type=types, blocking=False)
        if msg is None:
            break
        k = msg.get_type()
        t = acc.t(msg._timestamp)
        if k == "MODE":
            acc.set_mode(t, mavutil.mode_mapping_acm.get(msg.ModeNum, str(msg.Mode)), armed)
        elif k == "ARM":
            armed = bool(msg.ArmState)
            acc.set_mode(t, acc.mode or "?", armed)
        elif k == "MSG":
            acc.event(t, "msg", msg.Message.strip(), "warning" if ALERT.search(msg.Message) else "info")
        elif k == "CMD":
            acc.items[msg.CNum] = {"cmd": msg.CId, "p": [msg.Prm1, msg.Prm2, msg.Prm3, msg.Prm4],
                                   "lat": msg.Lat, "lon": msg.Lng, "alt": msg.Alt}
            if not acc.seq_changes or acc.seq_changes[-1][1] != msg.CNum:
                acc.seq_changes.append((t, msg.CNum))
        elif k == "POS":
            acc.pos.append((t, msg.Lat, msg.Lng, getattr(msg, "RelHomeAlt", msg.Alt), None))
        elif k == "ORGN" and msg.Type == 1 and msg.Lat:
            acc.home = (msg.Lat, msg.Lng)
        elif k == "GPS":
            acc.gps.append((t, msg.Status, msg.NSats, msg.HDop))
        elif k == "BAT":
            acc.batt.append((t, getattr(msg, "RemPct", -1), msg.Volt))
        elif k == "ERR":
            acc.event(t, "error", f"ERR subsystem {msg.Subsys} code {msg.ECode}", "warning")
        elif k == "EV":
            acc.event(t, "event", f"event {msg.Id}")
        elif k == "PARM":
            acc.params[msg.Name] = msg.Value
        elif k == "XKF4" and armed and getattr(msg, "SV", 0) > 1.0:      # EKF velocity variance
            acc.ekf_bad.append((t, f"EKF velocity variance high ({msg.SV:.2f})"))
    # speed from successive positions (POS carries none)
    pts = acc.pos
    for i in range(1, len(pts)):
        (t0, la0, lo0, a0, _), (t1, la1, lo1, a1, _) = pts[i - 1], pts[i]
        dt = t1 - t0
        if dt > 0:
            n, e = _ne((la0, lo0), la1, lo1)
            pts[i] = (t1, la1, lo1, a1, math.hypot(n, e) / dt)


# --------------------------------------------------------------------------- #
# findings
# --------------------------------------------------------------------------- #

def _mode_at(acc: _Acc, t: float) -> tuple:
    mode, armed = None, False
    for ts, m, a in acc.mode_spans:
        if ts > t:
            break
        mode, armed = m, a
    return mode, armed


def _stuck_episodes(acc: _Acc) -> List[dict]:
    out = []
    changes = acc.seq_changes + [(acc.t_last, None)]
    wp_radius = acc.params.get("WPNAV_RADIUS", WP_RADIUS_DEFAULT_M * 100) / 100.0 \
        if "WPNAV_RADIUS" in acc.params else acc.params.get("WP_RADIUS_M", WP_RADIUS_DEFAULT_M)
    for (t_a, seq), (t_b, _) in zip(changes, changes[1:]):
        if t_b - t_a < STUCK_MIN_S:
            continue
        mode, armed = _mode_at(acc, t_a + 1)
        if not armed or (mode or "").upper() not in AUTO_MODES:
            continue
        span = [p for p in acc.pos if t_a <= p[0] <= t_b]
        if len(span) < 3:
            continue
        slow = [p for p in span if p[4] is not None and p[4] < STUCK_SPEED_MPS]
        if len(slow) < 0.7 * len(span):
            continue
        it = acc.items.get(seq, {})
        cmd = it.get("cmd")
        name = NAV_NAMES.get(cmd, f"command {cmd}" if cmd is not None else "unknown item")
        last = span[-1]
        detail, why, fix = [], "", ""
        if it.get("lat") and acc.home and cmd in (16, 18, 19, 21):
            n_t, e_t = _ne(acc.home, it["lat"], it["lon"])
            n_v, e_v = _ne(acc.home, last[1], last[2])
            off = math.hypot(n_t - n_v, e_t - e_v)
            dz = last[3] - it["alt"]
            detail.append(f"{off:.1f} m from the item's position horizontally, {dz:+.1f} m in height")
        prev = acc.items.get(seq - 1, {})
        if cmd == 115 or prev.get("cmd") == 115:
            why = "a CONDITION_YAW was pending: the mission waits until the yaw is reached"
            fix = ("drop the 'yaw' item or follow it with the waypoint; check heading quality "
                   "(compass / ExtNav yaw) - a noisy heading never settles within 2 deg")
        elif cmd in (19, 93) or (cmd == 16 and it.get("p", [0])[0] >= t_b - t_a - 2):
            why = f"{name} holds by design ({it.get('p', [0])[0]:g} s)"
            fix = "expected - shorten hold_s / secs if unwanted"
        elif cmd == 17:
            why = "LOITER_UNLIM never ends on its own"
            fix = "use loiter_time instead"
        elif cmd in (20, 21):
            why = f"{name} in progress"
        elif seq == max(acc.items or {0: 0}) and cmd not in (20, 21):
            why = "the mission ended on this item without RTL/LAND, so the copter hovers in AUTO"
            fix = "end missions with 'rtl' or 'land' (rtl_at_end: true)"
        elif cmd == 16:
            bad_ekf = [e for e in acc.ekf_bad if t_a <= e[0] <= t_b]
            if bad_ekf:
                why = "the EKF position was not healthy, so the waypoint could not be tracked"
                fix = "check GPS / ExtNav (VIO) quality during this period; see the EKF findings"
            else:
                why = (f"the waypoint was never registered as reached (WP radius {wp_radius:g} m) "
                       "while the copter held position")
                fix = ("check the position estimate against the target (GPS-denied drift?), the "
                       "target altitude, and WPNAV_RADIUS / WP_RADIUS_M")
        out.append({"severity": "critical", "t": round(t_a, 1),
                    "title": f"Stuck for {t_b - t_a:.0f} s on mission item {seq} ({name}) in {mode}",
                    "detail": "; ".join(detail) or "position barely changed",
                    "likely_cause": why or "unknown - see the timeline around this time",
                    "fix": fix})
    return out


def _findings(acc: _Acc) -> List[dict]:
    f = _stuck_episodes(acc)
    for e in acc.timeline:
        if e["severity"] == "warning":
            f.append({"severity": "warning", "t": e["t"], "title": e["text"], "detail": "",
                      "likely_cause": "", "fix": ""})
    # unexpected autonomous -> RTL / LAND / LOITER transitions with no operator command visible
    for (t, mode, armed) in acc.mode_spans:
        if armed and (mode or "").upper() in ("RTL", "LAND", "SMART_RTL", "AUTO.RTL", "AUTO.LAND"):
            f.append({"severity": "info", "t": round(t, 1), "title": f"switched to {mode}",
                      "detail": "check the messages just before: failsafe or operator command?",
                      "likely_cause": "", "fix": ""})
    if acc.batt:
        low = [b for b in acc.batt if 0 <= b[1] <= 20]
        if low:
            f.append({"severity": "warning", "t": round(low[0][0], 1),
                      "title": f"battery down to {min(b[1] for b in low):.0f}%",
                      "detail": f"{min(b[2] for b in acc.batt):.2f} V minimum", "likely_cause": "", "fix": ""})
    if acc.gps:
        weak = [g for g in acc.gps if g[1] < 3 or g[2] < 6]
        if weak and acc.armed is not None:
            f.append({"severity": "warning", "t": round(weak[0][0], 1),
                      "title": f"weak GPS during the log ({len(weak)} samples: fix<3D or <6 sats)",
                      "detail": f"min sats {min(g[2] for g in acc.gps)}, max HDOP "
                                f"{max(g[3] for g in acc.gps):.1f}", "likely_cause": "", "fix": ""})
    if acc.ekf_bad:
        f.append({"severity": "warning", "t": round(acc.ekf_bad[0][0], 1),
                  "title": f"EKF trouble while armed ({len(acc.ekf_bad)} samples)",
                  "detail": acc.ekf_bad[0][1], "likely_cause": "", "fix": ""})
    gaps = [(a, b) for a, b in zip(acc.hb_times, acc.hb_times[1:]) if b - a > 3.0]
    if gaps:
        worst = max(gaps, key=lambda g: g[1] - g[0])
        f.append({"severity": "warning", "t": round(worst[0], 1),
                  "title": f"telemetry gaps: {len(gaps)} x >3 s without a heartbeat "
                           f"(worst {worst[1] - worst[0]:.0f} s)",
                  "detail": "radio link dropouts - commands sent then may not have arrived",
                  "likely_cause": "", "fix": "check antennas / range / the 3DR radio's air speed and ECC"})
    if acc.radio:
        rss = [r[1] for r in acc.radio]
        rem = [r[2] for r in acc.radio]
        errs = acc.radio[-1][5] - acc.radio[0][5]
        f.append({"severity": "warning" if min(rss + rem) < 60 or errs > 50 else "info", "t": 0,
                  "title": f"3DR radio: RSSI {min(rss)}-{max(rss)} local, {min(rem)}-{max(rem)} remote, "
                           f"{errs} rx errors",
                  "detail": "RSSI under ~60 means the link is near its range limit", "likely_cause": "", "fix": ""})
    return sorted(f, key=lambda x: ({"critical": 0, "warning": 1, "info": 2}[x["severity"]], x["t"]))


def analyze(path: str) -> dict:
    p = Path(path)
    acc = _Acc()
    kind = "dataflash" if p.suffix.lower() == ".bin" else "tlog"
    (_parse_bin if kind == "dataflash" else _parse_tlog)(str(p), acc)
    if acc.home is None and acc.pos:
        first = next((q for q in acc.pos if q[1]), acc.pos[0])
        acc.home = (first[1], first[2])
    mission = []
    for seq in sorted(acc.items):
        it = acc.items[seq]
        row = {"seq": seq, "cmd": it["cmd"], "name": NAV_NAMES.get(it["cmd"], str(it["cmd"])),
               "p": [round(v, 2) for v in it["p"]], "alt": round(it["alt"], 1)}
        if it.get("lat") and acc.home and seq > 0:
            n, e = _ne(acc.home, it["lat"], it["lon"])
            row.update(north_m=round(n, 1), east_m=round(e, 1))
        mission.append(row)
    path_xy = [[r["east_m"], r["north_m"]] for r in mission
               if "east_m" in r and r["cmd"] in (16, 18, 19)]
    alts = [r["alt"] for r in mission if r["cmd"] in (16, 18, 19)]
    track = []
    for q in acc.pos[::max(1, len(acc.pos) // 600)]:
        if acc.home and q[1]:
            n, e = _ne(acc.home, q[1], q[2])
            track.append([round(e, 1), round(n, 1), round(q[3], 1)])
    return {
        "file": p.name, "kind": kind, "autopilot": acc.autopilot,
        "duration_s": round(acc.t_last, 1), "home": acc.home,
        "findings": _findings(acc),
        "timeline": acc.timeline[:800],
        "mission": mission,
        "replay": {"points": path_xy, "alt": round(max(alts), 1) if alts else None},
        "track": track,
        "stats": {"positions": len(acc.pos), "mission_items": len(acc.items),
                  "radio_samples": len(acc.radio), "messages": len(acc.timeline)},
    }


if __name__ == "__main__":
    r = analyze(sys.argv[1])
    if "--json" in sys.argv:
        print(json.dumps(r, indent=2))
    else:
        print(f"{r['file']} ({r['kind']}, {r['autopilot']}, {r['duration_s']:.0f} s)")
        for x in r["findings"]:
            print(f"  [{x['severity']:8}] t={x['t']:7.1f}s {x['title']}")
            for k in ("detail", "likely_cause", "fix"):
                if x[k]:
                    print(f"             {k}: {x[k]}")
