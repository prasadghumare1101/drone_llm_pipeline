# Full simulation in one container (PX4 + ROS 2 Humble + Gazebo Classic GUI)

This image bakes the **entire stack** — nothing is installed on the host:

| Inside the image | Version |
|---|---|
| ROS 2 Humble (+ rviz2) | `osrf/ros:humble-desktop-full` |
| Gazebo Classic | 11 + `gazebo_ros` plugins |
| PX4-Autopilot SITL | pinned to `v1.17.0-alpha1` (commit `d2548ced9d`) |
| **Your modified iris** (front camera) | copied over PX4's stock model |
| MicroXRCE-DDS Agent | `v2.4.3` |
| px4_msgs | `main` (matches the pinned PX4) |
| The pipeline + aero_gcs backend | this repo |

The host provides only: the **Linux kernel**, an **X server** (to show the Gazebo
window), and optionally an **NVIDIA GPU** for smooth rendering.

> **Reality check.** This is a big image (~8–10 GB) and the first build takes
> **30–45 min** (PX4 compiles from source). It was authored carefully from PX4's
> canonical build recipe but has **not** been built on this machine (Docker isn't
> installed here) — budget one iteration on your first build.

---

## Build

```bash
cd ~/drone_llm_pipeline
DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.sim -t drone-llm-sim .
```
(BuildKit is required so `docker/Dockerfile.sim.dockerignore` is honoured. It's
the default in Docker 23+.)

## Run — with the Gazebo Classic GUI

```bash
xhost +local:root                 # once per host session: let the container use your display
export HF_TOKEN=hf_xxxxx          # optional, for the LLM
cd ~/drone_llm_pipeline/docker
./run_sim.sh                      # opens a shell inside the container
```

`run_sim.sh` wires up everything the GUI needs: the X11 socket, `DISPLAY`,
`XAUTHORITY`, `--network host`, and `--gpus all` **if** the NVIDIA container
toolkit is present (otherwise it falls back to software rendering, which still
works — just slower).

## Fly a mission (inside the container)

Open the shell with `./run_sim.sh`, then use these terminals (each is
`docker exec -it drone_sim bash`, or use `tmux` inside):

```bash
# terminal 1 — the DDS bridge
agent

# terminal 2 — PX4 SITL + Gazebo Classic GUI (window appears on your host)
sim empty 1              #  world=empty, 1 drone   (try 'sim warehouse 1')

# terminal 3 — run an LLM mission against it
mission "take off to 30 metres, fly a 40 metre square, then land"
```

`agent`, `sim`, `mission`, `dashboard` are shortcuts defined in
`docker/entrypoint.sh` — under the hood they are the same commands you already
know (`MicroXRCEAgent`, `make px4_sitl`, `run_pipeline.py`).

## The dashboard (optional)

```bash
# inside the container
dashboard                # control_api on :8000, video :8080, rosbridge :9090
```
Because the container uses `--network host`, open the separate frontend image
(or `npm run dev`) and point your browser at it — it reaches these ports on
localhost exactly as before.

---

## GPU rendering (recommended, optional)

For fluid Gazebo, install the NVIDIA container toolkit on the host **once**:
```bash
# NVIDIA driver must already be installed on the host
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
`run_sim.sh` auto-detects it and adds `--gpus all`. Without a GPU the sim uses
Mesa software rendering — correct, just lower frame-rate.

## Why this is "conflict-free"

The two classic PX4-in-Docker breakages are handled explicitly:
- **empy version** pinned to `3.3.4` (PX4 `main` fails to build against empy 4.x).
- **px4_msgs matches PX4** (`main` ↔ the pinned PX4 commit), so the uXRCE-DDS
  type hashes line up and topics actually deliver.
Plus the launch-reliability env this project already proved: the **online model
DB is disabled** (no "Gazebo won't open" hang) and the **pxh shell is off**.

## Portability

The image runs on **any x86_64 Linux** with Docker + an X server — Ubuntu,
Fedora, Arch, Debian, RHEL. The distro on the host does not matter; ROS/PX4/
Gazebo all live inside the container. For ARM64 hosts, rebuild with
`docker buildx --platform linux/arm64` (PX4 supports arm64).
