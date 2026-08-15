#!/usr/bin/env bash
# Bring up the DRONE simulation stack on the real rig (Ubuntu 22.04 + ROS 2 Humble).
# Terminals are used instead of one big launch file so PX4's pxh> shell stays usable.
#
#   Terminal A:  ./launch/px4_sitl_stack.sh agent      # uXRCE-DDS agent (UDP 8888)
#   Terminal B:  ./launch/px4_sitl_stack.sh sitl       # PX4 SITL + Gazebo Classic
#   Terminal C:  source /opt/ros/humble/setup.bash
#                source ~/px4_ros_ws/install/setup.bash   # px4_msgs workspace
#                python3 run_pipeline.py --prompt "Patrol the perimeter loop twice at 15 metres" --backend px4
#
# Env overrides: PX4_DIR (default ~/PX4-Autopilot), AGENT_PORT (default 8888)
set -euo pipefail
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
AGENT_PORT="${AGENT_PORT:-8888}"

case "${1:-}" in
  agent)
    echo "[stack] starting Micro XRCE-DDS agent on UDP ${AGENT_PORT}"
    exec MicroXRCEAgent udp4 -p "${AGENT_PORT}"
    ;;
  sitl)
    echo "[stack] starting PX4 SITL + Gazebo Classic from ${PX4_DIR}"
    cd "${PX4_DIR}"
    # PX4 >= v1.14 target name; on older tags use: make px4_sitl gazebo
    exec make px4_sitl gazebo-classic
    ;;
  mavsdk-note)
    echo "For the ROS-free path: PX4 SITL exposes MAVLink on udp://:14540;"
    echo "run:  python3 run_pipeline.py --prompt '...' --backend mavsdk"
    ;;
  *)
    echo "usage: $0 {agent|sitl|mavsdk-note}"; exit 1;;
esac
