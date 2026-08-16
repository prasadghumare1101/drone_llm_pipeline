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

# --- permanent HF token (safe: stays on the host, never inside the image) -----
# Set your token ONCE:   mkdir -p ~/.config/drone_llm
#                        echo 'HF_TOKEN=hf_xxxxx' > ~/.config/drone_llm/hf.env
# From then on every run picks it up automatically (no export needed).
TOKEN_FILE="${DRONE_LLM_TOKEN_FILE:-$HOME/.config/drone_llm/hf.env}"
if [ -z "${HF_TOKEN:-}" ] && [ -f "$TOKEN_FILE" ]; then
  # shellcheck disable=SC1090
  set -a; . "$TOKEN_FILE"; set +a
  echo "info: loaded HF_TOKEN from $TOKEN_FILE"
fi
[ -n "${HF_TOKEN:-}" ] || echo "warn: no HF_TOKEN -> the LLM falls back to the offline parser."

xhost +local:root >/dev/null 2>&1 || \
  echo "warn: 'xhost' failed — the Gazebo GUI needs an X server on the host."

# GPU passthrough. Disable entirely with AERO_NO_GPU=1 (software rendering).
# Different NVIDIA setups need different flags:
#   * a registered "nvidia" runtime  -> --runtime=nvidia   (works in CDI mode too,
#     where --gpus all is REJECTED with "please use --runtime=nvidia")
#   * otherwise, the classic         -> --gpus all
# GL_ENV forces the NVIDIA driver for RENDERING — without it the container falls
# back to Mesa/llvmpipe (software), the usual cause of a slow/laggy Gazebo window.
# __NV_PRIME_RENDER_OFFLOAD helps hybrid (Optimus) laptops whose X runs on Intel.
GPU_ARGS=()
GL_ENV=(--env __GLX_VENDOR_LIBRARY_NAME=nvidia
        --env __NV_PRIME_RENDER_OFFLOAD=1
        --env NVIDIA_DRIVER_CAPABILITIES=all)
if [ "${AERO_NO_GPU:-0}" = "1" ]; then
  echo "info: GPU disabled (AERO_NO_GPU=1) -> software rendering."
elif docker info 2>/dev/null | grep -A3 -i "runtimes:" | grep -qi nvidia; then
  GPU_ARGS=(--runtime=nvidia --env NVIDIA_VISIBLE_DEVICES=all "${GL_ENV[@]}")
  echo "info: NVIDIA GPU on via --runtime=nvidia (set AERO_NO_GPU=1 to disable)."
elif docker info 2>/dev/null | grep -qi nvidia; then
  GPU_ARGS=(--gpus all "${GL_ENV[@]}")
  echo "info: NVIDIA GPU on via --gpus all."
else
  echo "info: no NVIDIA runtime -> software rendering (slower, still works)."
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
