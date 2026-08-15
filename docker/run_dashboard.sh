#!/usr/bin/env bash
# ONE COMMAND to get the whole thing running from a pulled image:
#   ./run_dashboard.sh
# Then open http://localhost:3000/ and press START SIM. No other terminals.
#
# It gives the container everything it needs:
#   * X11 (so Gazebo Classic can open its window on your display)
#   * host networking (so the browser reaches :3000/:8000/:8080/:9090)
#   * the NVIDIA GPU if the container toolkit is present (else software render)
set -euo pipefail

# Pull from GHCR by default; override with IMAGE=... for a local build.
IMAGE="${IMAGE:-ghcr.io/prasadghumare1101/drone-llm-sim:latest}"

xhost +local:root >/dev/null 2>&1 || \
  echo "warn: 'xhost' failed — the Gazebo GUI needs an X server on the host."

GPU_ARGS=()
if docker info 2>/dev/null | grep -qi nvidia; then
  GPU_ARGS=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
  echo "info: NVIDIA GPU passthrough on."
else
  echo "info: no NVIDIA runtime -> Gazebo uses software rendering (slower, still works)."
fi

echo "info: pulling / starting $IMAGE …"
exec docker run --rm -it \
  --name drone_dashboard \
  --network host \
  --env DISPLAY="${DISPLAY:-:0}" \
  --env QT_X11_NO_MITSHM=1 \
  --env XAUTHORITY=/root/.Xauthority \
  --env HF_TOKEN="${HF_TOKEN:-}" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw" \
  "${GPU_ARGS[@]}" \
  "$IMAGE"
# (no command => the image's default CMD ["dashboard"] auto-starts everything)
