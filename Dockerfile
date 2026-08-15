# ============================================================================
#  drone_llm_pipeline — portable core image
#
#  What runs in this image:
#     prompt  ->  LLM (Hugging Face or offline)  ->  validate  ->  compile
#             ->  kinematic simulator  (deterministic, pure-Python)
#
#  What does NOT run here (needs ROS 2 / PX4 / Gazebo / hardware, not portable):
#     --backend px4 | mavsdk | nav2      and the aero_gcs dashboard.
#     Those are installed natively on the robot machine (see README).
#
#  The image is small on purpose: the only third-party dependency is jsonschema.
# ============================================================================

# Match the project's Python (3.10). "slim" = small Debian base, no build tools.
FROM python:3.10-slim

# Never write .pyc files and never buffer stdout — logs appear immediately.
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# All app files live under /app.
WORKDIR /app

# 1) Install dependencies FIRST (own layer) so Docker can cache them: they only
#    re-install when this line changes, not on every code edit.
#    jsonschema = the single runtime dependency of the core pipeline.
#    pytest     = tiny, lets the image verify itself (see DOCKER.md step 3).
RUN pip install --no-cache-dir "jsonschema>=4.17" "pytest>=7.0"

# 2) Copy ONLY the code the core pipeline needs (see .dockerignore for exclusions).
COPY run_pipeline.py    ./run_pipeline.py
COPY schema/            ./schema/
COPY llm_layer/         ./llm_layer/
COPY validator/         ./validator/
COPY executor/          ./executor/
COPY sim_bridge/        ./sim_bridge/
COPY tests/             ./tests/

# 3) Run as a non-root user (safer; the container never needs root).
RUN useradd --create-home pilot && chown -R pilot:pilot /app
USER pilot

# HF_TOKEN is NOT baked into the image (that would leak the secret). Pass it at
# RUN time:  docker run -e HF_TOKEN=hf_xxx ...   Without it, --llm auto falls
# back to the deterministic offline parser, so the container still works offline.

# The container IS the pipeline: `docker run <image> --prompt "..."` forwards
# straight to run_pipeline.py's arguments.
ENTRYPOINT ["python3", "run_pipeline.py"]

# With no arguments, run a safe offline demo instead of erroring.
CMD ["--prompt", "Patrol the perimeter loop twice at 15 metres", "--llm", "offline", "--dry-run"]
