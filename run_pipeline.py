#!/usr/bin/env python3
"""End-to-end pipeline driver.

    prompt ──llm_layer──> proposed JSON (UNTRUSTED string)
           ──validator──> MissionPlan  (frozen, typed)      <- trust boundary
           ──executor───> ExecutableCommand tuple (deterministic)
           ──sim_bridge─> vehicle / simulator

The vehicle NEVER executes raw LLM output: the executor's type gate only
accepts a MissionPlan, and every backend only accepts ExecutableCommands.

Examples:
    python3 run_pipeline.py --prompt "Patrol the perimeter loop twice at 15 metres"
    python3 run_pipeline.py --prompt "..." --backend px4        # real PX4 SITL
    python3 run_pipeline.py --mission-file tests/sample_missions/valid_perimeter_loop.json
    python3 run_pipeline.py --prompt "..." --dry-run            # stop after compile

Exit codes: 0 = completed, 2 = mission rejected by validator,
            3 = backend/execution failure, 4 = LLM failure.
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from llm_layer import LLMError, propose_mission_json            # noqa: E402
from validator import MissionValidationError, validate_mission  # noqa: E402
from executor import command_to_dict, commands_digest, compile_mission  # noqa: E402
from sim_bridge import get_backend                              # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LLM -> validated JSON -> "
                                             "deterministic executor -> sim/autopilot")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--prompt", help="natural-language mission request")
    src.add_argument("--mission-file", help="skip the LLM; validate this JSON file")
    ap.add_argument("--backend", default="sim",
                    choices=["sim", "px4", "mavsdk", "nav2"],
                    help="execution backend (default: sim = built-in kinematic simulator)")
    ap.add_argument("--llm", default="auto",
                    choices=["auto", "huggingface", "offline"],
                    help="LLM backend (auto: Hugging Face if HF_TOKEN set, "
                         "else the deterministic offline parser)")
    ap.add_argument("--out-dir", default=None,
                    help="artifact directory (default: runs/<UTC timestamp>)")
    ap.add_argument("--dry-run", action="store_true",
                    help="stop after compiling; print the command sequence")
    ap.add_argument("--llm-attempts", type=int, default=3,
                    help="re-ask the LLM this many times when it returns "
                         "malformed JSON or an off-schema plan (default: 3). "
                         "Safety rejections are never retried.")
    args, _ = ap.parse_known_args(argv)

    out_dir = Path(args.out_dir) if args.out_dir else \
        PROJECT_ROOT / "runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---- [1/4] + [2/4] propose, then validate — the trust boundary ----- #
    #
    # LLMs occasionally emit a malformed token (a dropped quote, a missing
    # comma). That is transient, so a bad *format* is re-asked rather than
    # fatal. A SAFETY rejection is never retried: re-rolling until the limits
    # happen to pass would launder the very request the operator must be told
    # was refused. Mission files skip retries entirely — the input is fixed.
    attempts = 1 if args.mission_file else max(1, args.llm_attempts)
    plan = None
    proposed = ""

    for attempt in range(1, attempts + 1):
        if args.prompt:
            suffix = "" if attempt == 1 else f" (attempt {attempt}/{attempts})"
            print(f"[1/4] LLM ({args.llm}) proposing mission for: {args.prompt!r}{suffix}")
            try:
                proposed = propose_mission_json(args.prompt, backend=args.llm)
            except LLMError as e:
                print(f"[1/4] LLM FAILED: {e}", file=sys.stderr)
                return 4
        else:
            print(f"[1/4] reading mission file (treated as untrusted): {args.mission_file}")
            proposed = Path(args.mission_file).read_text(encoding="utf-8")

        (out_dir / "proposed_mission.json").write_text(proposed, encoding="utf-8")
        print(f"      proposed JSON: {len(proposed)} bytes -> {out_dir/'proposed_mission.json'}")

        print("[2/4] validating against schema + safety rules …")
        try:
            plan = validate_mission(proposed)
            break
        except MissionValidationError as e:
            recoverable = e.stage in ("json", "schema")
            if recoverable and attempt < attempts:
                print(f"[2/4] {e.stage} error — re-asking the LLM "
                      f"({attempt + 1}/{attempts}):\n      {e.errors[0]}",
                      file=sys.stderr)
                continue
            print(f"[2/4] {e}", file=sys.stderr)
            (out_dir / "rejection.txt").write_text(str(e), encoding="utf-8")
            return 2
    (out_dir / "validated_mission.json").write_text(
        json.dumps(json.loads(proposed), indent=2, sort_keys=True), encoding="utf-8")
    print(f"      PASS: '{plan.mission_name}' vehicle={plan.vehicle} "
          f"cruise={plan.cruise_speed_mps} m/s  source_sha256={plan.source_sha256[:16]}…")

    # ---- [3/4] Compile — deterministic, no LLM ------------------------- #
    print("[3/4] compiling to executable command sequence …")
    cmds = compile_mission(plan)
    digest = commands_digest(cmds)
    (out_dir / "compiled_commands.json").write_text(
        json.dumps({"digest_sha256": digest,
                    "commands": [command_to_dict(c) for c in cmds]}, indent=2),
        encoding="utf-8")
    print(f"      {len(cmds)} commands, digest {digest[:16]}… "
          f"(same JSON in => same digest, every time)")
    if args.dry_run:
        for i, c in enumerate(cmds):
            print(f"      {i:3d}: {command_to_dict(c)}")
        print("[dry-run] stopping before execution.")
        return 0

    # ---- [4/4] Execute on the chosen backend --------------------------- #
    print(f"[4/4] executing on backend '{args.backend}' …")
    try:
        backend = get_backend(args.backend, out_dir=out_dir)
        result = backend.run(cmds)
    except (RuntimeError, ValueError, TypeError) as e:
        print(f"[4/4] backend error: {e}", file=sys.stderr)
        return 3

    status = "COMPLETED" if result.completed else "FAILED"
    print(f"\n=== MISSION {status} on '{result.backend}' ===")
    print(f"  {result.detail}")
    for k, v in result.metrics.items():
        print(f"  {k:22s} {v}")
    for name, path in result.artifacts.items():
        print(f"  artifact {name:13s} {path}")
    print(f"  run artifacts dir     {out_dir}")
    return 0 if result.completed else 3


if __name__ == "__main__":
    raise SystemExit(main())
