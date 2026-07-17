# Stage 3 Plan — Classical Baseline + Residual RL

Written 2026-06-30. Covers Stage 3 of the RL pipeline (`rl_pipeline_doc_v0_3.md`
§3, §7): adding the velocity-setpoint wrapper, the classical formation
controller, the FORMATION task, and the residual RL action interface on top
of the Stage 1 baseline.

> **Progress update (2026-07-16).** The task breakdown below is kept as the
> original plan of record; this is what has actually landed. See
> `progress_log.md` for detail.
>
> | Task | Status |
> | --- | --- |
> | Blocker: classical controller source missing | **Resolved** — reconstructed in `stirling/controller/`, MuJoCo-validated |
> | (a) velocity-setpoint wrapper | **Done, merged** (PR #4). Proven inert at `control_mode=0` by byte-identical checkpoints |
> | (b) classical controller C port | **Done** — `ocean/drone/velocity_controller.h`. NFR-36 passes in-env: reform 1.25 s, min sep 0.488 m (MuJoCo ref 1.29 / 0.49) |
> | (c) FORMATION task | **Done** — `FORMATION` in `tasks.h`: 5-mode slot geometry, Rz(yaw) rotation, blend, 3-part feedforward velocity, waypoint-cursor centroid. In-env cruise tracking 0.073 m |
> | (d) observation extension | **Done** — 23 → 41 floats (RPMs still last). Neighbours are real at `num_drones=4`, zeros at 1; `u_classic` wired in per §2.5 |
> | (e) reward extension | **Done** — velocity now measured vs the target (identity for static-target tasks); jerk, 2 s-alignment and separation terms added, all inert by default. Alpha re-tune deferred to the 3b sweep |
> | (f) Stage 3a classical benchmark | **Done** — `bash stirling/tests/run_stage3a_bench.sh`. Floor at n=4: score 970.19, perf 0.9716, ema_dist 0.0217, tracking 0.036 m, 0 oob / 0 breaches. Box-only, per the competition brief |
> | (g) Stage 3b residual | Next — `--env.task 2 --env.num-drones 4 --env.control-mode 1 --env.k-res <k>`. **Judge on `ema_dist`, not `perf`** (see progress_log) |
>
> Two deviations from this doc worth knowing:
>
> - **Training is via Modal** (`stirling/modal/`), not the RunPod flow this doc
>   assumes. One command; see `stirling/modal/README.md`.
> - **`BASE_K_MOT` was lowered 0.15 → 0.05 s** — a Stage 2 platform
>   recalibration taken early. At 0.15 s the actuator lag capped the control
>   cascade and made the 2 s reform rule unreachable regardless of the control
>   law. This doc's assumption that Stage 3 "runs independently of the hardware
>   decision using the existing Crazyflie constants" held for task (a) but
>   **broke at task (b)** — one platform constant had to move for the formation
>   requirement to be achievable at all.

## Status going in

Stage 1 is complete: unmodified Ocean `drone` HOVER task trained to 40M
steps on RunPod, baseline metrics recorded in `stage1_handoff.md` (score
836.9, ema_dist 0.020). `ocean/drone/` (`binding.c`, `drone.c`, `drone.h`,
`dronelib.h`, `tasks.h`) is still byte-identical to upstream — no formation,
controller, or obstacle code exists yet. This doc is the breakdown for the
next increment.

Stage 2 (platform recalibration — replacing the Crazyflie `BASE_*` constants
in `dronelib.h` with the selected hardware's measured values) is **blocked**
on hardware platform selection (tech doc §3, still open). Per RL pipeline
doc §6.1 ("Hardware platform decision slips and blocks Stage 2"), Stage 3a
and the early part of Stage 3b are explicitly designed to run independently
of that decision, using the existing Crazyflie constants and stubbed
(zeroed) neighbour observations. This plan follows that sequencing. Re-run
against real platform constants once Stage 2 unblocks.

## Blocker to flag before starting: classical controller source is missing

Tech doc §8 describes a MATLAB/Octave draft (`default_params.m`,
`formation_tracking.m`, `safety_filter.m`, `classical_control_step.m`,
`formation_manager.m`, `demo_formation.m`) as complete and smoke-tested
(reform 0.60 s, min separation 0.54 m). Neither this repo nor
`stirling-drone-project` contains those files — they don't exist anywhere
under `~/GIT`. Before NFR-36 (C port) can start, either locate the original
MATLAB/Octave files (wherever they were written — not committed to git) and
add them to this repo, or reconstruct the control law directly from tech
doc §8.2/§8.3 and re-validate against the recorded smoke-test numbers. This
is a prerequisite for task (b) below, not a Stage 3 nice-to-have.

## Task breakdown

Each task lists the files it touches and the approximate team-hours from
RL pipeline doc §7.

### (a) Velocity-setpoint wrapper / control stack — ~15 hrs

Add the four-layer control stack inside `c_step` (RL pipeline §2.3, §2.5):

```
u_classic = formation_controller(target_offset, vel, neighbours)   // every tick
u_total   = clip(u_classic + k_res * dv, -v_max, +v_max)           // RL residual
v_cmd     = safety_filter(u_total, ...)                            // APF default
motor_rpms = velocity_to_motor_pid(v_cmd)                          // lowest layer
```

- New file: `ocean/drone/velocity_controller.h` (or extend `dronelib.h`).
- `c_step` in `dronelib.h` currently calls `move_drone(agent, &env->actions[4*i])`
  directly with the native 4-float motor action — this is the hook point to
  intercept and replace with the new stack when the policy's action space is
  the 3-float residual.
- Passing `dv = {0,0,0}` (or an empty residual) must recover pure classical
  control — this is the Stage 3a benchmark mode and the runtime fallback
  (tech doc §10).
- Tune in the order specified by tech doc §8.5: (1) velocity-tracking inner
  loop, (2) classical controller Kp/Ki, (3) safety filter params, (4) only
  then enable the residual.

### (b) Classical controller C port — ~15 hrs (blocked on the missing MATLAB source above)

- Port `formation_tracking.m` → nominal law `u = Kff*v_target + Kp*(p_target-p) + Ki*∫e dt`.
- Port `safety_filter.m` → APF repulsion (inter-drone, obstacle, course
  boundary) with closing-rate damping; CBF-QP is an optional upgrade, not
  required for Stage 3.
- Port `classical_control_step.m` → the four-layer composition in (a).
- NFR-36 validation: unit test the C port against the MATLAB smoke-test
  results — reform ≤2.0 s (target 0.60 s), min separation ≥0.40 m floor
  (target 0.54 m) — before Stage 3 work proceeds past 3a.

### (c) FORMATION task scaffolding — ~20 hrs

- `ocean/drone/tasks.h`: extend the `DroneTask` enum (currently `IDLE,
  HOVER, ORBIT, FOLLOW, CUBE, CONGO, FLAG, RACE`) with `FORMATION`, add it
  to `TASK_NAMES`, and add `set_target_formation()` following the existing
  `set_target_cube` / `set_target_flag` pattern but driven by the centroid +
  heading-rotated slot offset (tech doc §4.3): `target = centroid.p +
  Rz(centroid.yaw) * slot_offset(mode, slot)`.
- Slot offset lookup table for the 5 modes (square/box home, line, stack,
  compressed, diamond), per tech doc §4.2/§4.3 — port from
  `formation_manager.m` once located.
- Rule-based path planner: advances a centroid waypoint cursor, exposes
  current mode and time-to-next-mode-change to the observation builder.
- For Stage 3 specifically (single-agent, stub neighbours): the FORMATION
  task only needs to hold a programmatically moving centroid target —
  multi-agent slot assignment and mode transitions are Stage 4 scope.

### (d) Observation builder extension — ~15 hrs

`compute_drone_observations()` in `dronelib.h` currently writes a fixed
23-float vector (body velocity ×3, body omega ×3, quaternion ×4,
two-scale target offset ×6, target normal ×3, RPMs ×4 — RPMs must stay
last). Extend to ~38 floats (41 with `u_classic`):

- +9: three neighbour relative positions, body frame, sorted by ID. Stub as
  zeros for Stage 3 (real neighbours are Stage 4).
- +5: formation mode one-hot.
- +1: time-to-next-mode-change, tanh-scaled.
- +TBD: obstacle feature slot — Stage 3 placeholder can be omitted or
  zeroed; real distance/bearing-to-next-gate lands with the obstacle
  primitives in Stage 4.
- +3 (optional): `u_classic` appended so the policy sees the baseline it's
  correcting. Wire in from the start per RL pipeline §2.5; ablate in 3b if
  it doesn't help.

### (e) Reward extension — folded into (a)/(c) time estimates

- Re-evaluate `alpha_dist`/`alpha_hover`/`alpha_shaping`/`alpha_omega`
  (currently in `DroneEnv` / `config/drone.ini`) against the FORMATION
  target instead of the static HOVER target.
- Add reform-within-2s shaping: 200-step (2 s @ 100 Hz) exponential bonus
  window after a mode-change event.
- Add graduated penalty for time-since-aligned exceeding 2 s.
- Light jerk penalty on velocity-setpoint deltas (less load-bearing than
  under direct motor control, since the autopilot/velocity PID filters
  short-timescale noise).
- Richer termination set in `c_step` (currently just `oob`/`timeout`): add
  obstacle contact and inter-drone separation breach, logged to `Log`.

### (f) Stage 3a — classical-only benchmark run

`task = FORMATION`, `dv = 0`, `num_agents = 1`, stub neighbours. Record
metrics as the floor the residual policy must beat. No RL training in this
run — it's a sim validation pass on the ported controller.

### (g) Stage 3b — enable residual, train to convergence

3-float action space (`acts = env.single_action_space.shape[0]` becomes 3,
matching `FormationPolicy` in RL pipeline §4.1). `k_res` starts at 1.0 and
sweeps alongside reward weights (RL pipeline §6.2 open question). Pass
criterion: residual policy measurably improves on the 3a benchmark.

## Sequencing

1. Locate or reconstruct the classical controller source (blocker, see
   above).
2. (a) velocity wrapper scaffold with `dv=0` passthrough, wired to the
   *existing* HOVER task first — confirms the wrapper alone doesn't break
   Stage 1's converged behaviour before any controller logic lands.
3. (b) classical controller C port + NFR-36 unit test.
4. (c) FORMATION task + (d) observation extension, in parallel once (b)
   passes.
5. (f) Stage 3a benchmark.
6. (g) Stage 3b residual training.

## Open questions carried into Stage 3

- Reward weights (including new reform terms) — sweep at start of Stage 3
  using PufferLib Protein/sweep tooling (RL pipeline §6.2).
- `k_res` tuning — start 1.0, sweep during 3b.
- `u_classic` observation — wire in, ablate if it doesn't help.
- CBF-QP vs APF — APF is the Stage 3 default; CBF stays a later upgrade.

## References

- `rl_pipeline_doc_v0_3.md` §2.3–2.5, §3, §7 (this repo, `stirling/docs/`)
- `drone_project_docs_v0_6.md` §8 (classical controller), §4 (formation
  architecture) — this repo, `stirling/docs/`
- `requirements_doc_v0_4.md` NFR-34–37 — this repo, `stirling/docs/`
- `ocean/drone/dronelib.h`, `tasks.h`, `drone.h` (current upstream-identical
  state)

**Note:** `stirling/docs/` now carries the June revisions in markdown —
tech doc v0.6, requirements v0.4, and RL pipeline v0.3 — with the
superseded v0.5/v0.3/v0.2 docs removed. The classical controller draft
(§8) has since been reconstructed and validated in MuJoCo; see
`stirling/controller/`.
