#!/usr/bin/env bash
# Entrypoint for the full-sim image: make ROS 2 + the workspace + PX4 available
# in every shell, then run whatever command was passed (default: bash).
#
# Convenience sub-commands:
#   sim [world] [n]   launch PX4 SITL + Gazebo Classic (GUI) with the modified iris
#   agent             run the MicroXRCE-DDS agent (udp4 :8888)
#   dashboard         run the aero_gcs control_api backend (:8000)
#   mission "<text>"  run one LLM->validate->PX4 mission via run_pipeline.py
#   <anything else>   executed as-is
set -e

source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash
export PYTHONPATH="/opt/drone_llm_pipeline:${PYTHONPATH:-}"
# Gazebo finds the modified iris here; online DB stays disabled (set in Dockerfile).
export GAZEBO_MODEL_PATH="/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH:-}"

cmd="${1:-bash}"
case "$cmd" in
  sim)
    world="${2:-empty}"; n="${3:-1}"
    echo "[sim] launching PX4 SITL + Gazebo Classic (world=$world, drones=$n)"
    # single drone -> make px4_sitl; multi -> sitl_multiple_run
    cd /opt/PX4-Autopilot
    if [ "$n" -le 1 ]; then
      # empty/default use the bare target; a named world uses the __<world> suffix.
      if [ "$world" = "empty" ] || [ "$world" = "default" ]; then
        exec make px4_sitl gazebo-classic_iris
      else
        exec make px4_sitl "gazebo-classic_iris__${world}"
      fi
    else
      exec Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n "$n" -w "$world"
    fi
    ;;
  agent)
    exec MicroXRCEAgent udp4 -p 8888
    ;;
  dashboard)
    # Auto-start EVERYTHING the operator needs — no extra terminals:
    #   * control_api  (:8000) — orchestrates the sim + missions
    #   * the dashboard UI (:3000) — static site
    # The user then opens the browser and clicks START SIM; control_api brings
    # up the uXRCE agent, PX4 SITL + Gazebo, rosbridge, telemetry and video.
    echo "[dashboard] starting control_api (:8000) …"
    python3 /opt/drone_llm_pipeline/aero_gcs/backend_services/control_api.py --force \
        > /tmp/control_api.log 2>&1 &
    # If the backend dies, take the container down too (so failures are visible).
    api_pid=$!
    sleep 2
    echo "────────────────────────────────────────────────────────"
    echo "  AEGIS / AERO-GCS dashboard  →  http://localhost:3000/"
    echo "  (open it, pick a world, press START SIM to launch Gazebo)"
    echo "────────────────────────────────────────────────────────"
    # Serve the built UI in the foreground (keeps the container alive).
    cd "${FRONTEND_DIR:-/opt/frontend_dist}"
    exec python3 -m http.server 3000 --bind 0.0.0.0
    ;;
  mission)
    shift
    exec python3 /opt/drone_llm_pipeline/run_pipeline.py --prompt "$*" --backend px4 --llm huggingface
    ;;
  *)
    exec "$@"
    ;;
esac
