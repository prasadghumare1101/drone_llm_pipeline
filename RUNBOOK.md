# RUNBOOK — operating the pipeline on the real rig (Ubuntu 22.04, PX4 main checkout)

Two configurations. Start with **A** (first clean pipeline flight), then move to
**B** (pipeline + your RTAB-Map perception stack in the same session).

Ground rules that prevent every known conflict on this machine:

1. **Exactly one offboard commander.** The pipeline owns the vehicle. Never run
   a QGC mission at the same time — QGC may stay open as a *spectator* only.
2. Every terminal that talks ROS sources BOTH overlays, in this order:
   `source /opt/ros/humble/setup.bash && source ~/px4_ros_ws/install/setup.bash`
   (px4_msgs branch must match the PX4 checkout: main <-> main).
3. Same `ROS_DOMAIN_ID` everywhere (leave it unset = 0 on all terminals).
4. If the graph acts stale after restarts: `ros2 daemon stop`, and if FastDDS
   SHM misbehaves: kill agent + SITL, `rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing*`, restart.
5. Start order: agent -> SITL -> (perception) -> pipeline.
   Stop order: pipeline (or let it finish) -> perception -> SITL (Ctrl-C in pxh) -> agent.
6. API token: `export HF_TOKEN=hf_...` in the pipeline terminal (or ~/.bashrc)
   — never in files. Without it the pipeline auto-falls back to the offline parser.

---

## Configuration A — first LLM -> PX4 flight (3 terminals + optional QGC)

**T1 — uXRCE-DDS agent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**T2 — PX4 SITL + Gazebo Classic** (either of your working targets)
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris__warehouse
# or, with your Nashik world + downward camera model:
# PX4_SITL_WORLD=${HOME}/.gazebo/worlds/nashik.world make px4_sitl gazebo-classic_iris_downward_depth_camera
```
Wait for: T1 flooding with `create_datawriter`, and `pxh> INFO [commander] Ready for takeoff!`.
First run only, in `pxh>` (persists in parameters.bson afterwards):
```
param set COM_RCL_EXCEPT 4        # SITL has no RC; clears the arming objection
```

**T3 — pipeline**
```bash
source /opt/ros/humble/setup.bash && source ~/px4_ros_ws/install/setup.bash
export HF_TOKEN=hf_...            # hf.co/settings/tokens ('Make calls to
                                  # Inference Providers' permission; free credits)
cd ~/drone_llm_pipeline
python3 run_pipeline.py --prompt "Patrol the perimeter loop twice at 15 metres" --backend px4 --dry-run
python3 run_pipeline.py --prompt "Patrol the perimeter loop twice at 15 metres" --backend px4
```
Success signature: `position feedback live on /fmu/out/vehicle_local_position_v1`
-> `[1/12] Arm` -> Offboard + armed -> square flown twice at 15 m -> RTL ->
auto-disarm -> `MISSION COMPLETED ... 8/8 waypoints`.

**T4 (optional) — QGC spectator.** Watch mode/track only. Do not upload/start missions.

---

## Configuration B — pipeline + full perception (RTAB-Map, RViz) — 6 terminals

T1 and T2 exactly as in Configuration A (use the model/world with the sensors
you need; `iris_downward_depth_camera` for the geo-localizer camera).

**T3 — static TFs** (needed by RTAB-Map/RViz only; the pipeline itself never uses TF)
```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0.15 0 0.05 0 0 0 base_link front_camera_link &
ros2 run tf2_ros static_transform_publisher 0.15 0 0.1  0 0 0 base_link lidar_link &
ros2 run tf2_ros static_transform_publisher 0    0 0.15 0 0 0 base_link lidar_3d_link &
```

**T4 — RViz**
```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

**T5 — RTAB-Map** (your proven invocation)
```bash
source /opt/ros/humble/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  frame_id:=base_link \
  subscribe_scan:=true  scan_topic:=/scan \
  subscribe_rgb:=true   rgb_topic:=/front_camera/image_raw \
  camera_info_topic:=/front_camera/camera_info \
  approx_sync:=true visual_odometry:=false icp_odometry:=true \
  args:="-d --Reg/Strategy 1 --RGBD/NeighborLinkRefining true"
```

**T6 — pipeline** (same as Configuration A's T3). Example geo-localizer-altitude
mission, now legal under the 60 m ceiling:
```bash
python3 run_pipeline.py --prompt "Patrol the perimeter loop twice at 55 metres" --backend px4
```

Why B has no conflicts: RTAB-Map/RViz/TF are pure consumers on the ROS graph —
none of them publish `/fmu/in/*` or command the vehicle, so the pipeline remains
the sole offboard authority. The only shared resource is CPU: with lockstep SITL
the sim's Real Time Factor may drop under RTAB-Map load, which is harmless
(everything slows coherently). If RTF collapses badly, run RTAB-Map on the
recorded bag afterwards instead of live.

---

## Troubleshooting quick hits

| Symptom | Cause / fix |
|---|---|
| bridge stuck "Warming up", then warns "no position feedback" | px4_msgs branch != PX4 checkout -> rebuild `~/px4_ros_ws` on matching branch; or agent/SITL session not established (T1 must flood on SITL boot) |
| arming refused | read reason in `pxh>`; usually `COM_RCL_EXCEPT 4` not set |
| `ros2 topic list` empty for /fmu | agent not running when SITL booted -> restart SITL after agent; `ros2 daemon stop` |
| mission validates but nothing moves | a QGC mission owns the vehicle -> land, clear it, rerun pipeline |
| weird discovery after many restarts | clean `/dev/shm/fastrtps_*` with agent+SITL down |
