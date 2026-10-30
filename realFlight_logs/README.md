# realFlight_logs

Logs from real ArduPilot flights, for diagnosing failures (e.g. the vehicle
stopping and hovering mid-mission) and reproducing them in simulation.

Put one sub-folder per flight, e.g. `2026-10-29_field_A_stuck_at_wp5/`, with:

| File | Where it comes from | Why |
|---|---|---|
| `*.BIN` (dataflash log) | flight controller SD card, `APM/LOGS/` (or Mission Planner / QGC: download logs) | **most important** - MSG, MODE, CMD, NTUN, XKF, BAT, RCIN at full rate |
| `*.tlog` | Mission Planner / QGC telemetry logs, or mavlink-router on the Pi | what went over the 3DR 915 MHz radio (link quality, dropped commands) |
| mission JSON | `~/vloc/missions/received/` on the Pi | exactly what was uploaded |
| `notes.txt` | you | what you expected, what happened, roughly when (time into the flight) |

Contents of this folder (except this README) are not committed to git.
