# drone_llm_pipeline

Prompt → LLM → **validated** mission JSON → **deterministic** executor → simulator / autopilot.

The vehicle only ever executes a fixed path compiled from *validated* JSON — never raw LLM
output. This is enforced structurally, not by convention:

```
 natural language
       │
 ┌─────▼──────┐   raw string (UNTRUSTED)
 │ llm_layer/ │──────────────┐   no execution, no parsing into trusted objects
 └────────────┘              │
                     ┌───────▼────────┐
                     │   validator/   │   JSON Schema + safety rules
                     │ (trust boundary│   → frozen MissionPlan, or loud
                     │   — the ONLY   │     MissionValidationError listing
                     │  code touching │     every specific problem
                     │  raw LLM text) │
                     └───────┬────────┘
                     ┌───────▼────────┐
                     │   executor/    │   pure function: MissionPlan → tuple of
                     │ (deterministic)│   ExecutableCommands. TypeError on any
                     │                │   string/dict. Loops unrolled here.
                     └───────┬────────┘
        ┌────────────┬───────┴──────┬──────────────┐
   ┌────▼───┐   ┌────▼────┐   ┌─────▼────┐   ┌─────▼────┐
   │  sim   │   │   px4   │   │  mavsdk  │   │   nav2   │      sim_bridge/
   │kinematic│  │uXRCE-DDS│   │ MAVLink  │   │ TB3+Nav2 │
   └────────┘   └─────────┘   └──────────┘   └──────────┘
```

Isolation guarantees (tested in `tests/`):

* `executor.compile_mission()` **raises `TypeError`** on anything that is not a
  `validator.MissionPlan` — raw LLM strings and unvalidated dicts cannot compile.
* Every backend **raises `TypeError`** on anything that is not a compiled
  `ExecutableCommand` sequence.
* Same validated JSON in ⇒ same command tuple and same SHA-256 digest out, every time
  (`executor.commands_digest`, proven in `tests/test_executor.py`).
* Cross-layer imports flow one way only:
  `llm_layer` → (nothing) · `executor` → `validator` (types only) · `sim_bridge` → `executor`.

---

## STEP 0 — Environment verification (this build sandbox, 2026-07-05)

Probed with real commands (`ros2 --version`, `printenv ROS_DISTRO`, `gazebo --version`,
`ros2 pkg list | grep nav2`, `which MicroXRCEAgent`, repo path checks):

| Component            | Required          | Found in sandbox            | Status | Action taken / required |
|----------------------|-------------------|-----------------------------|--------|--------------------------|
| Ubuntu 22.04         | 22.04 LTS         | **24.04.4 LTS**             | FAIL   | Not fixable in-place: ROS 2 **Humble is 22.04-only** (24.04 pairs with Jazzy). Target your 22.04 rig. |
| ROS 2 Humble         | `ros2` + `ROS_DISTRO=humble` | not installed    | FAIL   | Install commands below (needs `packages.ros.org`, unreachable from this sandbox). |
| PX4-Autopilot + SITL | repo + built SITL | no checkout                 | FAIL   | Clone/build commands below (multi-GB build; not sensible on this 1-vCPU sandbox). |
| Gazebo Classic       | `gazebo --version` (11.x) | not installed       | FAIL   | Comes with `ros-humble-gazebo-ros-pkgs` / PX4 setup script on 22.04. EOL upstream — pinned versions below. |
| Micro XRCE-DDS Agent | binary/service    | not installed               | FAIL   | Snap or source-build commands below. |
| Nav2                 | `ros2 pkg list \| grep nav2` | unavailable (no ROS 2) | FAIL | `apt` commands below (**listed, not auto-run**, per brief). |
| Python 3 + pip       | ≥3.8              | 3.12.3 / pip 24.0           | PASS   | — |
| jsonschema / pytest / matplotlib | for core pipeline | installed via pip | PASS | `pip3 install -r requirements.txt` |
| HF_TOKEN             | optional          | not set                     | INFO   | Pipeline auto-falls back to the deterministic offline LLM backend. |

**Decision made:** since the ROS/PX4/Gazebo stack is neither present nor installable in this
sandbox, the pipeline ships with a **built-in deterministic kinematic simulator backend**
(`--backend sim`) so the *complete* pipeline runs and is CI-tested anywhere, plus fully
implemented PX4 uXRCE-DDS, MAVSDK and Nav2 bridges (reviewed, guarded imports, exact
bring-up commands below) for the real rig.

---

## Quickstart (any machine, no ROS needed)

```bash
cd drone_llm_pipeline
pip3 install -r requirements.txt          # jsonschema, pytest, matplotlib

python3 -m pytest tests/ -v               # 27 tests: validator, executor, end-to-end

# Full pipeline in the built-in simulator:
python3 run_pipeline.py \
    --prompt "Patrol the perimeter loop twice at 15 metres" \
    --backend sim --out-dir runs/demo
```

Expected output (verified in this build):

```
[2/4] PASS: 'Patrol the perimeter loop twice at 15 metres' vehicle=drone cruise=5.0 m/s
[3/4] 12 commands, digest 161b8006e4f99aed…
=== MISSION COMPLETED on 'sim' ===
  8/8 waypoints reached, 361.2 m in 88.2 s (sim), landed+disarmed
```

Artifacts land in `runs/demo/`: `proposed_mission.json` (raw LLM output),
`validated_mission.json`, `compiled_commands.json` (+ digest), `trajectory.csv`,
`sim_report.json`, `trajectory_plot.png` (top-down loop + altitude profile).

Unsafe requests are rejected loudly *before* anything moves (exit code 2):

```bash
python3 run_pipeline.py --prompt "Patrol the perimeter once at 500 metres" --backend sim
# Mission REJECTED (5 problem(s)):
#   - $.commands[0] (TAKEOFF): altitude 500.0 m exceeds hard ceiling 30.0 m
#   ...
```

Other useful invocations:

```bash
python3 run_pipeline.py --prompt "..." --dry-run                # print compiled commands, execute nothing
python3 run_pipeline.py --mission-file tests/sample_missions/valid_perimeter_loop.json  # bypass LLM
export HF_TOKEN=hf_...          # https://hf.co/settings/tokens -> fine-grained token
                                # with 'Make calls to Inference Providers' permission
                                # (free monthly inference credits on free accounts)
# Token hygiene: set this in ~/.bashrc or the shell only. NEVER paste tokens
# into chats, commits, or files in this repo — if a token is ever exposed,
# revoke it immediately at hf.co/settings/tokens. The code reads env vars only.
python3 run_pipeline.py --prompt "..." --llm huggingface        # or --llm auto
# auto: Hugging Face if HF_TOKEN is set, else the offline parser.
# The validator gate is identical for every backend.
```

Model default: `openai/gpt-oss-120b` on the Hugging Face Inference Providers
router (`HF_MODEL` to override — provider/policy suffixes like
`openai/gpt-oss-120b:cheapest` work). Retired-model 404s walk an automatic
fallback chain, and if nothing is callable the error lists the models your
token CAN call, so the fix is always one env var. Browse models:
https://huggingface.co/docs/inference-providers

---

## Real-rig setup (Ubuntu 22.04) — exact commands, **not auto-run here**

### ROS 2 Humble
```bash
sudo apt install software-properties-common curl && sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
     sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

### PX4 SITL + Gazebo Classic
```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot -b v1.14.3
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh     # installs Gazebo Classic 11 + toolchain on 22.04
cd ~/PX4-Autopilot && make px4_sitl gazebo-classic   # first build takes a while
```

### Micro XRCE-DDS Agent (either)
```bash
sudo snap install micro-xrce-dds-agent --edge
# or from source:
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig /usr/local/lib/
```

### px4_msgs workspace (for the DDS bridge)
```bash
mkdir -p ~/px4_ros_ws/src && cd ~/px4_ros_ws/src
git clone https://github.com/PX4/px4_msgs.git        # match branch to your PX4 tag
cd ~/px4_ros_ws && source /opt/ros/humble/setup.bash && colcon build
```

### Nav2 + TurtleBot3 (ground-robot case)
```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
     ros-humble-nav2-simple-commander ros-humble-turtlebot3-gazebo
# verify:  ros2 pkg list | grep nav2
```

---

## Running against the real simulators

### Drone: PX4 SITL over uXRCE-DDS (3 terminals)
```bash
# A) agent
./launch/px4_sitl_stack.sh agent            # MicroXRCEAgent udp4 -p 8888
# B) PX4 + Gazebo Classic
./launch/px4_sitl_stack.sh sitl             # make px4_sitl gazebo-classic
# C) pipeline (source ROS + px4_msgs first)
source /opt/ros/humble/setup.bash && source ~/px4_ros_ws/install/setup.bash
python3 run_pipeline.py --prompt "Patrol the perimeter loop twice at 15 metres" --backend px4
```
PX4 ≥ v1.16 renamed some `/fmu/out` topics with a `_v1` suffix — pass
`Px4DdsBackend(status_topic="/fmu/out/vehicle_status_v1")` if status never arrives.

### Drone without ROS: MAVSDK straight into SITL
```bash
pip3 install mavsdk        # SITL already exposes MAVLink on udp://:14540
python3 run_pipeline.py --prompt "..." --backend mavsdk
```

### Ground robot: Gazebo Classic + Nav2
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch launch/nav2_sim.launch.py headless:=False
# other sourced terminal:
python3 run_pipeline.py \
  --prompt "Drive the ground robot around a 10 metre square box three laps" --backend nav2
```

---

## Conventions & documented defaults

| Item | Default | Where defined |
|------|---------|---------------|
| Frame | `LOCAL_ENU_METERS`: x = East, y = North, `alt` = m above home (up +). Home = (0,0,0). | `schema/mission_schema.json` |
| Units | metres, m/s, seconds | schema |
| Command whitelist | `TAKEOFF, GOTO, LOOP, HOLD, LAND, RTL` | schema (`oneOf` + `additionalProperties:false`) |
| Drone limits | alt 2–60 m, ≤12 m/s, geofence \|x\|,\|y\| ≤ 200 m, loops ≤10, ≤500 unrolled wps | `validator/safety_limits.py` |
| Ground limits | alt = 0, ≤1.5 m/s, geofence ≤ 100 m | `validator/safety_limits.py` |
| Mission shape | drone: starts `TAKEOFF`, ends `LAND`/`RTL`; ground: `GOTO/LOOP/HOLD` only | validator semantic rules |
| "perimeter" | square with corners (±20, ±20) | LLM system prompt + offline backend |
| ENU→NED (PX4) | north = y, east = x, down = −alt | `px4_dds_bridge.py`, `mavsdk_bridge.py` |
| Sim dynamics | point-mass, 2 m/s climb, 1 m/s descent, 0.5 m acceptance radius, dt = 0.05 s | `sim_bridge/kinematic_sim.py` |

Known, deliberate limitations: per-`GOTO` speed is honoured exactly in the kinematic sim;
under PX4 the position setpoints are tracked at `MPC_XY_CRUISE`/`MPC_XY_VEL_MAX` (validator
has already capped requested speeds). ArduPilot (e.g. a SpeedyBee F405 stack) would need a
pymavlink GUIDED-mode backend — add it as another `MissionBackend` subclass in `sim_bridge/`;
nothing upstream changes.

## Layout

```
schema/      mission_schema.json           the LLM↔executor contract
llm_layer/   prompts.py, llm_client.py     proposes JSON only (Hugging Face router or offline)
validator/   mission_validator.py, safety_limits.py   trust boundary
executor/    commands.py, mission_executor.py         deterministic compiler
sim_bridge/  kinematic_sim.py, px4_dds_bridge.py, mavsdk_bridge.py, nav2_bridge.py
tests/       27 tests + sample_missions/
launch/      px4_sitl_stack.sh, nav2_sim.launch.py
run_pipeline.py                            CLI: the only place the layers meet
```
