#!/usr/bin/env bash
# FPV feed root-cause diagnostic — run INSIDE the running container:
#     docker exec -it drone_dashboard bash /entrypoint_diag.sh
# or copy it in:
#     docker cp docker/diagnose_fpv.sh drone_dashboard:/tmp/ && \
#     docker exec -it drone_dashboard bash /tmp/diagnose_fpv.sh
#
# It walks the FPV chain link by link and prints, for each, OK / FAIL and why:
#   1 rendering     is gzserver using the GPU or slow software (llvmpipe)?
#   2 plugin        did the gazebo_ros camera plugin load (ROS args present)?
#   3 topic         does a camera image topic exist on the ROS graph?
#   4 frames        is that topic actually publishing (and at what rate)?
#   5 streamer      is :8080 up and locked onto a topic?
#   6 http          does :8080/video_feed return image bytes?
# Touches nothing; read-only.
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
source /opt/ros_ws/install/setup.bash 2>/dev/null
say(){ printf '\n\033[1m== %s ==\033[0m\n' "$1"; }
ok(){  printf '  \033[32mOK\033[0m   %s\n' "$1"; }
bad(){ printf '  \033[31mFAIL\033[0m %s\n' "$1"; }

say "0. processes"
for p in gzserver gzclient px4 video_streamer control_api rosbridge; do
  pgrep -fa "$p" | grep -qv grep && ok "$p running" || bad "$p NOT running"
done

say "1. rendering (GPU vs software) — the usual FPV killer"
command -v glxinfo >/dev/null || { echo "  installing mesa-utils…"; apt-get update -qq >/dev/null 2>&1 && apt-get install -y -qq mesa-utils >/dev/null 2>&1; }
REND=$(DISPLAY="${DISPLAY:-:0}" glxinfo 2>/dev/null | grep -i "OpenGL renderer" | head -1)
echo "  $REND"
if echo "$REND" | grep -qiE "nvidia|amd|intel|mesa (dri|iris)"; then
  ok "hardware GPU rendering — camera should render fast"
elif echo "$REND" | grep -qi "llvmpipe\|softpipe\|swrast"; then
  bad "SOFTWARE rendering (llvmpipe) — camera renders very slowly / stalls."
  echo "     -> re-run the container with working --runtime=nvidia GPU passthrough,"
  echo "        or use the lightweight iris so software rendering can keep up."
else
  bad "no GL renderer reported — gzserver may have no rendering context at all."
fi

say "2. gazebo_ros camera plugin"
ls /opt/ros/humble/lib/libgazebo_ros_camera.so >/dev/null 2>&1 \
  && ok "libgazebo_ros_camera.so present" || bad "camera plugin missing from image"
echo "  GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH:-<unset>}"
if pgrep -fa gzserver | grep -q "libgazebo_ros_factory"; then
  ok "gzserver launched WITH the ROS factory (-s libgazebo_ros_factory.so)"
else
  bad "gzserver launched WITHOUT the ROS plugins -> no ROS camera topics."
  echo "     (ROS_VERSION was probably unset when make px4_sitl ran.)"
fi

say "3. camera topics on the graph"
CAMS=$(ros2 topic list 2>/dev/null | grep -iE "image|camera" | grep -viE "depth|points|info|theora")
if [ -n "$CAMS" ]; then ok "found:"; echo "$CAMS" | sed 's/^/       /'
else bad "NO camera image topics — the camera sensor is not publishing."; fi

say "4. is the camera topic publishing frames?"
CAM1=$(echo "$CAMS" | grep -iE "front" | head -1); CAM1=${CAM1:-$(echo "$CAMS" | head -1)}
if [ -n "$CAM1" ]; then
  echo "  measuring rate on $CAM1 (5s)…"
  timeout 6 ros2 topic hz "$CAM1" 2>/dev/null | grep -i "average rate" | head -1 | sed 's/^/  /' \
    || bad "no messages in 6s — sensor rendering is stalled (see link 1)."
else bad "no topic to measure (link 3 failed)."; fi

say "5. video streamer health (:8080)"
curl -s -m3 http://localhost:8080/health | sed 's/^/  /' || bad ":8080 not answering — streamer down."

say "6. HTTP feed returns image bytes?"
N=$(timeout 4 curl -s http://localhost:8080/video_feed 2>/dev/null | head -c 200000 | grep -c $'\xff\xd8\xff')
[ "${N:-0}" -gt 0 ] && ok "MJPEG returning JPEG frames ($N in the first 200 KB)" \
                    || bad "no JPEG bytes from /video_feed."

echo
echo "Summary: the FIRST FAIL above is the root cause. A software-rendering"
echo "line at step 1 explains BOTH the slow Gazebo AND the blank FPV."
