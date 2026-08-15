# Running drone_llm_pipeline in Docker

The image contains the **portable core**: `prompt → LLM → validate → compile →
kinematic sim`. It does **not** contain ROS 2 / PX4 / Gazebo or the `aero_gcs`
dashboard — those need hardware/ROS and are installed natively on the robot.

Only dependency inside the image: `jsonschema`. Base: `python:3.10-slim`.
Expected image size: ~150 MB.

---

## 0. Install Docker (this machine does not have it yet)

```bash
sudo apt update && sudo apt install -y docker.io
sudo usermod -aG docker $USER      # run docker without sudo
# log out/in (or: newgrp docker) so the group takes effect
```

## 1. Build the image

```bash
cd ~/drone_llm_pipeline
docker build -t drone-llm-pipeline .
```

## 2. Run it

The container **is** the pipeline — arguments after the image name go straight
to `run_pipeline.py`.

**Offline (no token, deterministic):**
```bash
docker run --rm drone-llm-pipeline \
  --prompt "Patrol the perimeter loop twice at 15 metres" --llm offline --dry-run
```

**With the real LLM (pass your token at run time — never baked into the image):**
```bash
docker run --rm -e HF_TOKEN=hf_xxxxx drone-llm-pipeline \
  --prompt "Survey a 120 by 120 metre field at 50 m, photograph each pass, then RTL" \
  --llm huggingface --dry-run
```

**Run a full kinematic simulation and keep the artifacts on your machine:**
```bash
mkdir -p out
docker run --rm -v "$PWD/out:/app/runs" drone-llm-pipeline \
  --prompt "Climb to 40 m, fly a 5-point star with 80 m arms, then land" \
  --llm offline --backend sim
# trajectory.csv + sim_report.json appear in ./out/<timestamp>/
```

**No arguments → a safe offline demo runs (see the Dockerfile CMD).**

## 3. Run the tests inside the image (optional)

```bash
docker run --rm --entrypoint python3 drone-llm-pipeline -m pytest tests/ -q
```

---

## Notes

- `HF_TOKEN` is supplied with `-e HF_TOKEN=...` (or `--env-file .env`). It is
  intentionally **not** in the image, so the image is safe to share/push.
- Trajectory PNG plots need matplotlib, which is left out to keep the image
  small; the sim detects this and skips the plot (CSV + JSON still written).
  To include plots, add `matplotlib>=3.5` to the `pip install` line in the
  Dockerfile and rebuild.
- To run the PX4/ROS backends or the dashboard, use the native install on the
  robot machine — those are out of scope for this portable image.
