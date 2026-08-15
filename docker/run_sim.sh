#!/usr/bin/env bash
# Launch the full-sim container with everything the Gazebo Classic GUI needs:
# X11 socket, DISPLAY, and (if present) the NVIDIA GPU for smooth rendering.
#
#   ./run_sim.sh                 # open a shell in the container
#   ./run_sim.sh sim empty 1     # straight into PX4 SITL + Gazebo GUI
#
# Works on any Linux host running an X11 server. Run once per session on the host:
#   xhost +local:root      (allow the container to draw on your display)
set -euo pipefail

IMAGE="${IMAGE:-drone-llm-sim}"

# Let the container talk to the host X server (safe, local only).
xhost +local:root >/dev/null 2>&1 || \
  echo "warn: could not run 'xhost' — is an X server running on the host?"

# Use the NVIDIA runtime only if the toolkit is installed (else software render).
GPU_ARGS=()
if docker info 2>/dev/null | grep -qi nvidia; then
  GPU_ARGS=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
  echo "info: NVIDIA GPU passthrough enabled"
else
  echo "info: no NVIDIA runtime -> software rendering (works, just slower)"
fi

exec docker run --rm -it \
  --name drone_sim \
  --env DISPLAY="${DISPLAY:-:0}" \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw" \
  --env XAUTHORITY=/root/.Xauthority \
  --env HF_TOKEN="${HF_TOKEN:-}" \
  --network host \
  "${GPU_ARGS[@]}" \
  "$IMAGE" "$@"
