# Running the AERO-GCS dashboard with Docker

The dashboard has two halves, and only one belongs in a container.

```
┌─ FRONTEND (dockerized) ───────────┐     ┌─ BACKEND + SIM (native on host) ─┐
│  React/Vite UI served by nginx    │     │  control_api.py   :8000          │
│  → portable, one small image      │ ──► │  video_streamer   :8080          │
│                                   │     │  rosbridge        :9090          │
│  browser calls the host services  │     │  telemetry_node                  │
│  at localhost:8000/8080/9090      │     │  PX4 SITL + Gazebo (GUI, X11)    │
└───────────────────────────────────┘     └──────────────────────────────────┘
```

## Why the backend is NOT containerized

`control_api.py` is a **host orchestrator**: it runs `make px4_sitl` from
`~/PX4-Autopilot`, sources the host's `/opt/ros/humble`, launches the
MicroXRCEAgent, and opens the **Gazebo Classic GUI** on your `:0` display.
To do any of that from a container it would need host networking, the host PID
namespace, host mounts (`~/PX4-Autopilot`, ROS), and the X11 socket — i.e. a
container that is really just a host process in disguise. That adds fragility
for zero benefit, so the backend + sim stay native (run them as you do today).

The **frontend** has no such needs: it is static files, and the browser (which
runs on your host) reaches the backend at `localhost` directly. So only the
frontend is dockerized.

---

## 1. Build the frontend image

```bash
cd ~/drone_llm_pipeline/aero_gcs/frontend
docker build -t aero-gcs-frontend .
```
Expected size ~50 MB (nginx:alpine + the static build).

## 2. Start the backend + sim NATIVELY (unchanged)

```bash
# terminal 1 — the orchestrator
cd ~/drone_llm_pipeline/aero_gcs/backend_services
python3 control_api.py --force
```
(That one service launches the agent, PX4 SITL + Gazebo, rosbridge, telemetry
and video when you press START SIM in the UI.)

## 3. Serve the dashboard from the container

```bash
docker run --rm -p 3000:80 aero-gcs-frontend
```
Open **http://localhost:3000/**. The UI loads from the container; every button
and telemetry stream talks to the native backend on `localhost:8000/8080/9090`.

> Port 3000 is just where nginx serves the UI. Do NOT use 8000/8080/9090 for it
> — those belong to the native backend the browser calls.

---

## One-command option (docker-compose)

`aero_gcs/docker-compose.yml` builds + runs the frontend for you:

```bash
cd ~/drone_llm_pipeline/aero_gcs
docker compose up --build      # UI on http://localhost:3000/
```
The backend still runs natively (step 2) — compose only owns the frontend, on
purpose, for the reasons above.

## Rebuild after UI changes

The image is a static snapshot of the build. After editing the React code:
```bash
docker build -t aero-gcs-frontend aero_gcs/frontend   # or: docker compose up --build
```
(During active UI development, `npm run dev` on the host is faster — use Docker
for a packaged, shareable dashboard.)
