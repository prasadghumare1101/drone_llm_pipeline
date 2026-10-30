# Interactive (step-by-step) flight mode

One-shot mode (`run_pipeline.py`) turns one prompt into a whole mission:
arm → take off → fly → land. Interactive mode turns **each chat message into
one step** that starts from wherever the drone is *now*. After the step the
drone hovers in place and replies **"Done. What next?"**.

```
operator ─chat─► control_api ─proxy─► session_service (long-lived, owns the vehicle link)
                                         │
                ┌────────────────────────┼─────────────────────────────┐
                │ fast path (no LLM)     │ step path                   │
                │ stop/hold/land/home/   │ LLM(history + live pose)    │
                │ resume/status          │  → validate_step(pose)      │  ← trust boundary
                │ → link.hold/land/rtl   │  → compile_step             │
                │   instantly            │  → StepRunner on the link   │
                └────────────────────────┴─────────────────────────────┘
                                         │
                       VehicleLink: KinematicLink (sim) | Px4Link (PX4 SITL, uXRCE-DDS)
```

## Rules

| Requirement | How it is enforced |
|---|---|
| Each prompt is one step from the live position | The LLM gets the live pose + heading + armed/in-air state every turn. `validate_step()` resolves relative `MOVE` commands against that pose snapshot, so the model never does the arithmetic. |
| Hover and ask "Done. What next?" after each step | The step never ends in a landing unless the operator asked for one. The link holds position (PX4 OFFBOARD position hold) between steps. |
| Memory across messages | The session keeps the conversation (operator text + a factual summary of what was flown) and sends the last turns to the LLM with every request. |
| stop / hold / land / return-home act instantly | `session.commands.parse_fast_command()` is a deterministic regex parser that runs **before** the LLM. A match acts on the vehicle in the HTTP request thread, interrupts the running step and discards any plan still being generated. |
| The pilot always arms | The session and the step runner **never** arm. A take-off step waits in `AWAITING_ARM` until the pilot arms (RC or the dashboard ARM button). The pilot's arm is the launch approval. |
| No reply for 2 minutes → return home | Watchdog: airborne + idle (hovering, not executing, not planning) for `INACTIVITY_TIMEOUT_S = 120` → RTL. |
| Battery check before each step | `session.battery.check_step()` estimates the step's energy plus the trip home and refuses the step if battery is unknown, below the floor, or would end under the reserve. Fast commands are never blocked by battery. |
| RC always overrides | If the vehicle leaves OFFBOARD while armed and we did not ask for it (pilot flipped the mode switch / moved the sticks), the session enters `RC_OVERRIDE`: the step is cancelled, every AI command is refused (including land/home, since those would take control back), and the watchdog is suspended. Control returns only when the pilot hands it back (switch back to OFFBOARD, or say "resume"). |

## Step validation (`validator.validate_step`)

Same schema, same numeric limits and geofence as one-shot missions, plus:

* `MOVE` (new): `{"type":"MOVE","forward":0,"right":-5,"up":0}` (body frame,
  relative to the drone's heading) or `{"north":..,"east":..,"up":..}` (world
  frame). Body and world axes cannot be mixed in one MOVE. Resolved to an
  absolute GOTO from the pose snapshot, then fence- and altitude-checked.
* Airborne: `TAKEOFF` is refused. On the ground: the first flight command must
  be `TAKEOFF`.
* `LAND`/`RTL` are allowed only as the final flight command, and are not required.
* `compile_step()` emits no `Arm`; arming belongs to the pilot.

## Session states

`ON_GROUND → AWAITING_ARM → EXECUTING → HOVERING → (EXECUTING …) → RETURNING/LANDING → ON_GROUND`,
plus `PLANNING` (LLM in flight; the drone keeps hovering) and `RC_OVERRIDE`.
While a step is executing, non-fast messages are refused ("say stop first"),
so a new instruction never races the current one.

## Documented assumptions

* Interactive mode holds a constant heading so "left/right/forward" stay
  consistent from step to step. Body-frame MOVEs use the heading at the start
  of the step.
* Battery model: `DRAIN_PCT_PER_MIN = 2.5` in the air, 20 % reserve at landing,
  30 % minimum to start a step. Tune per airframe.
* PX4 keeps its own failsafes (offboard loss → `COM_OBL_RC_ACT`, low battery,
  geofence). The session is an extra layer on top of them.
