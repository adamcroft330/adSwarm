# Progress Log

Reverse-chronological record of significant project increments. Each entry
captures what changed, why, and what it unblocks. Complements the planning
docs (`stage3_plan.md`, `rl_pipeline_doc_v0_3.md`) which describe intent;
this records what actually landed.

---

## 2026-07-16 — FORMATION task (task c): formation manager C port + moving-centroid task

Ported `stirling/controller/formation_manager.py` into `ocean/drone/tasks.h`
and wired a `FORMATION` task into the env. Stage 3 tasks (a), (b), (c) are now
done; next is (d), the observation extension.

### What landed

- **`FORMATION` task** appended to the `DroneTask` enum (`TASK_NAMES`:
  `"formation"`, index 8). `set_target(...)` now takes the env's `Formation*`;
  all three call sites (reset, in-step reset, render task-cycling) updated.
- **Formation manager (tech doc §4.2–§4.3)** — slot-offset tables for all 5
  modes (box home, line, stack, compressed, diamond), `Rz(yaw)` heading
  rotation, linear mode-transition blend over `FM_BLEND_TIME`, and the 3-part
  feedforward velocity: `v_target = centroid.v + Rz(yaw)*d_off_blend +
  omega x r_off`. Geometry constants match `FormationParams` in
  `default_params.py` and are `-D`-overridable like the cascade gains.
- **Rule-based centroid planner** — a waypoint cursor: random waypoints inset
  2 m from the world extent (covers the largest slot offset), cruise at
  `FM_CRUISE_SPEED` with an arrival ease-in, heading slewed toward the course
  under a 0.5 rad/s turn-rate limit. `centroid.vel`/`yaw_rate` are set to the
  rates *actually applied* each tick, which is what keeps the slot
  feedforward exact.
- **`c_step` integration** — the centroid advances once per tick, then every
  slot target (position *and* velocity) refreshes before control runs. The
  task (b) tracking law consumes `target->vel` via `KFF` unchanged — the
  feedforward path was ready, as planned.
- Units note: `FORMATION` writes `target->vel` in m/s (what the velocity stack
  expects). The upstream `IDLE`/`CONGO` convention treats `target->vel` as a
  per-tick displacement (`move_target`) — the two differ by `ACTION_DT` and
  must not be mixed. `FORMATION` never calls `move_target`.

### Scope held to Stage 3

Single-formation, slots assigned round-robin by agent index, mode held at box
in-env. Multi-agent slot assignment and the mode-change scheduler are Stage 4;
the blend machinery is ported in full and test-covered now so the Stage 4
scheduler only has to call `formation_set_mode()`.

### Tests (4 new gates in `run_velocity_tests.sh`)

- **Geometry**: all 5 modes' pairwise spacings match the reference; tightest
  (compressed, 0.72 m) clears `VC_D_ACT` 0.70 m, so the APF never fights a
  held formation.
- **Blend**: analytic — midpoint offsets, blend-rate velocity, rotated frame,
  and completion all exact vs the Python reference.
- **Feedforward invariant**: `d(p_target)/dt == v_target` to 0.0013 m/s over
  ~12k checks spanning cruise, turns, waypoint captures, and two mode
  transitions. The one excluded tick is blend-exit, where alpha clips to 1
  partway through a step — piecewise in the Python reference too. This
  invariant failing is how a wrong yaw-rate or blend-rate term would surface.
- **In-env**: `task=FORMATION`, 4 drones, 8 s of waypoint cruising — tracking
  error 0.073 m (gate < 0.30), min separation 1.198 m (the box side, i.e.
  slots held exactly).

All prior gates unchanged and green (NFR-36: reform 1.25 s, min sep 0.488 m).

---

## 2026-07-16 — Decision: BASE_K_MOT 0.15 → 0.05 s; NFR-36 now met in-env

Resolves the blocker from the C-port entry below. **Decision taken: lower the
sim's motor time constant to a realistic value rather than accept a ~6.4 s
classical floor.** The controller then meets spec in the env with the shipped
config.

### Rationale

`BASE_K_MOT = 0.15 s` was inherited from upstream and is not defensible: a real
Crazyflie 2.1 is ~0.02–0.05 s, and the project targets 5"/250-class propulsion
(with bidirectional-DShot ESCs already mandated), which is faster still. The
constant was an artifact, and it was the single thing making the 2 s reform rule
unreachable — not the control law, which reproduces the MuJoCo reference
exactly once the actuator can serve it. Fixing a wrong constant is not the same
as tuning to green.

This is a **Stage 2 (platform recalibration) change taken early**. It is
directionally safe: every candidate platform is faster than 0.15 s, so 0.05 s
is closer to any hardware the team picks than the old value was.

### What changed

- `dronelib.h`: `BASE_K_MOT` 0.15 → **0.05 s**.
- `velocity_controller.h`: cascade raised to the **reference gains** the
  Python/MuJoCo rig validated — `KP` 0.6→2.0, `KV` 2→5, `KR` 12→200,
  `KW` 5→25, `V_MAX` 2→3, `KI` 0.1→0.3. These now match
  `stirling/controller/default_params.py` exactly, so the C and Python
  configurations have converged. Motors and gains had to move together; either
  alone fails (see the witness table below).

### Result: the gate passes in-env

| Metric | Before (k_mot 0.15) | Now (k_mot 0.05) | MuJoCo ref |
| --- | ---: | ---: | ---: |
| Formation reform | 6.41 s ✗ | **1.25 s** ✓ | 1.29 s |
| Min separation | 0.012 m ✗ | **0.488 m** ✓ | 0.49 m |
| Worst error | 2.55 m | **0.76 m** | 0.76 m |
| Hover settle | 3.66 s | **1.11 s** | — |
| Moving-target lag | 0.063 m | **0.010 m** | — |

`bash stirling/tests/run_velocity_tests.sh` — all gates pass. The motor-lag
witness table is retained in the suite: if anyone raises `BASE_K_MOT` again, it
shows the cost immediately (at 0.15 s these gains diverge to ~190 m).

### Consequences

- **The Stage 1 baseline is retired, not re-established.** Score 740.424 /
  `ema_dist` 0.099 was measured against 0.15 s motors and does not describe
  this env. It is *not* worth re-recording as a reference: its original job
  ("the regression target for later work", per `stage1_handoff.md`) is already
  spent — the wrapper was proven inert by byte-identical checkpoints — and the
  numbers Stage 3 is actually judged against are the **Stage 3a classical
  floor** and the **Stage 3b residual**, which must be measured on the
  FORMATION task, not motor-level HOVER. A `kmot005-smoke` run was done only to
  confirm the env still trains after the constant change — it does, and every
  metric improved as predicted (faster motors make the control problem easier):

  | Metric | k_mot 0.15 | k_mot 0.05 |
  | --- | ---: | ---: |
  | score | 740.4 | 820.2 |
  | ema_dist | 0.099 | **0.009** |
  | episode_length | 997.3 | **1024.0** (full) |
  | oob rate | 3.2% | **0%** |

  Hover is ~11× tighter and no episode goes out of bounds. Recorded as a smoke
  result, **not** a reference. (It lands near the original bf16 836.9 by
  coincidence — different precision *and* different dynamics, so the two are
  not comparable.) The older `wrapper-regression` checkpoint is historical.
- **A motor-level ceiling run is still worth doing later — on FORMATION.**
  RL pipeline §2.4 lists conditions to escalate to motor-level control. Judging
  that needs the ceiling (what unconstrained control achieves) alongside the
  3a floor and the 3b residual, all on the same task and dynamics:
  `classical (3a) < residual (3b) <= motor-level (ceiling)`. The 3a→3b gap is
  the residual's value; the 3b→ceiling gap is the cost of the architecture, and
  the escalation trigger. A motor-level *HOVER* number does not serve this —
  wrong task.
- **The published artifact overstates its conclusion.** Its "0.15 s is the
  blocker" claim rests on a two-point gain axis, not a search — the ladder
  theory predicts no stable config meets 2 s at 0.15 s, but that was never
  measured. The decision above makes the question moot in practice; the claim
  should still be softened if the artifact is shared onward.

### Known limitation, quantified (not a defect)

The APF cannot guarantee separation against a **head-on convergence above
~1.75 m/s** per drone (`D_ACT*KV/2`): each drone coasts `v/KV` after its command
reverses, so two closing head-on cover `2v/KV` = 1.2 m, exceeding the 0.70 m
activation band. This is inherent to distance-based APF with a saturated output,
it reproduces the Python reference faithfully, and tech doc §8.3 already claims
a hard guarantee only for the **CBF-QP upgrade** — this quantifies when that
upgrade becomes necessary. The formation scenario stays well inside the
envelope (min sep 0.488 m). Retained in the suite as a characterization, not a
gate.

## 2026-07-16 — Classical controller C port (task b); sim motor lag blocks NFR-36

Ported the classical controller from `stirling/controller/` (Python) into
`ocean/drone/velocity_controller.h`, replacing task (a)'s placeholder P-law.
**The port is faithful. The sim's motor constant, not the port, is what fails
the 2 s reform rule.**

### What landed

- **Tracking law (§8.2)** — `u = KFF*v_target + KP*e + KI*integ` with
  conditional anti-windup and a per-axis integral clamp. Per-drone integral
  state added to `Drone` (`dronelib.h`), reset in `init_drone`.
- **APF safety filter (§8.3)** — inter-drone repulsion with closing-rate
  damping, plus course-boundary push-back against the world extent. No solver
  dependency. Obstacle repulsion is deliberately absent: the env has no
  obstacle primitives yet (Stage 4); the Python term slots in unchanged.
- **Four-layer composition** — `u_classic → + k_res*dv → safety_filter →
  velocity_to_motor`. The residual is added *before* the filter, so the
  separation guarantee holds regardless of the policy. `u_classic` is returned
  for the future observation extension (RL pipeline §2.5).
- Cascade gains are now `-D`-overridable — the seam Stage 2 recalibration and
  the test sweep both use.

### The port is faithful — evidence

Run the reference cascade (the gains `stirling/controller/default_params.py`
uses: kp=2.0, kv=5.0, kr=200, kw=25, v_max=3.0) on a platform with a realistic
motor lag, and the C port reproduces the MuJoCo validation almost exactly:

| Metric | MuJoCo (Python) | C port @ `k_mot=0.05 s` |
| --- | ---: | ---: |
| Reform after 2.5 m/s kick | 1.29 s | **1.25 s** |
| Min inter-drone separation | 0.49 m | **0.487 m** |
| Worst formation error | 0.76 m | **0.76 m** |

This is the NFR-36 gate, and it **passes** —
`bash stirling/tests/run_velocity_tests.sh` asserts it (exit code).

### The real blocker: `BASE_K_MOT = 0.15 s`

With the sim's shipped Crazyflie constants the same law gives **reform 6.41 s**
and blows through the separation floor on the transient. The cause is the motor
time constant, which caps the whole cascade:

- At `k_mot=0.15 s` the reference cascade is **unstable** (diverges to ~190 m).
  The motor lag cannot support kp=2/kv=5, so the gains must be detuned to
  kp=0.6/kv=2 — and at kv=2 a 2.5 m/s kick coasts ~1.25 m before stopping,
  which no amount of outer-loop tuning recovers.
- Detuned gains + faster motors is *also* not enough (reform 3.75 s at
  `k_mot=0.02`): motors and gains are coupled, and both must move together.
- At any **realistic** motor lag (0.02–0.08 s) with the reference cascade, the
  gate passes. `BASE_K_MOT = 0.15 s` is sluggish for a Crazyflie 2.1 (real
  ≈ 0.02–0.05 s) and looks like an upstream sim artifact.

**Consequence:** the classical controller alone cannot meet the 2 s reform rule
in the env as shipped. This is the Stage 2 (platform recalibration) dependency
biting Stage 3 — the requirement was written for the real 250-class platform,
not the Crazyflie constants the sim inherits. Options, none taken yet: lower
`BASE_K_MOT` (a Stage 2 decision; it changes env dynamics and would invalidate
the recorded Stage 1 baseline), or accept a ~6.4 s classical floor for Stage 3a
and let the residual close the gap (its stated job, but 6.4 s → 2 s is a large
ask). **Flagging for a decision rather than silently tuning to green.**

### Also verified

- Feedforward works: 0.063 m lag tracking a 0.5 m/s target (≈0.83 m with
  `KFF=0`), so the moving-formation case is covered.
- APF equilibrium is correct — two drones commanded to the same point settle
  1.11 m apart. The floor violations are *transient* blow-through during a
  fast approach, not a logic error.
- Task (a) hover tests still pass, slightly improved by the I + feedforward
  terms (settle 3.66 s vs 4.12 s).

## 2026-07-16 — Stage 3a regression: velocity wrapper proven inert (task a closed)

Ran the `control_mode=0` regression the Stage 3 plan sequences before any
formation logic lands, now that the Modal framework and the wrapper are on one
branch. **Result: pass, conclusively.**

### Method

Two 40M-step HOVER runs via Modal, identical config (fp32, A10G, `seed=42`,
`config/drone.ini` defaults), differing *only* in whether the wrapper code is
present:

| Run | Branch | Wrapper in `c_step`? |
| --- | --- | --- |
| `wrapper-regression` | `stage3a-velocity-wrapper` | yes (inert, `control_mode=0`) |
| `control-no-wrapper` | `stirling-drone` | no — code absent entirely |

### Result: byte-identical

Both runs produced **byte-for-byte identical checkpoints**
(`md5 14794a4ab236d48fe8fb2938edd3d80e`) and identical metrics to three
decimals: score 740.424, `ema_dist` 0.099, `episode_return` 45.418,
`episode_length` 997.307, `perf` 0.937. Training is seeded and deterministic,
so identical weights after 40M steps proves the executed code path is
bit-identical — `control_mode=0` cannot regress Stage 1.

### Why the control run mattered

Against the *historical* Stage 1 baseline (score 836.9, `ema_dist` 0.020) the
wrapper run looks ~12% worse — which would read as a regression. It is not:
that baseline was **bf16 on an RTX 5090**, this is **fp32 on an A10G**, so two
variables moved alongside the wrapper. The no-wrapper control lands on exactly
740.424 too, attributing the entire gap to precision + GPU and none of it to
the wrapper. `ema_dist` 0.099 also matches the documented fp32 expectation
(~0.10) from the 2026-07-15 precision finding. Comparing against the old
baseline alone would have been ambiguous at best and misleading at worst.

Fallback checkpoint kept at `stirling/artifacts/drone/wrapper-regression/`
(fp32 40M HOVER, `.bin` + Mac-evalable `.pt`); other experiment outputs are
gitignored as regenerable.

**Stage 3 task (a) is closed** — the wrapper is verified inert by construction
*and* by experiment (PR #4).

## 2026-07-15 — Precision finding: fp32 default; Stage 1 HOVER verified end-to-end

Ran the first full HOVER baselines through the Modal framework and chased down
why a checkpoint that hovered on Modal looked broken when eval'd on the Mac.

### Root cause: bf16-trained policy does not transfer to fp32

The native backend trains in bf16 by default. A bf16-trained 40M HOVER policy
hovers under bf16 (ema_dist ~0.03, full ~1000-step episodes) but destabilises
within ~100 steps under fp32 (ema_dist ~2.6) — and the Mac/torch eval path
(`--slowly`) is fp32-only. So the same weights looked good on Modal (native
bf16) and bad on the Mac (torch fp32).

- **Conversion exonerated.** A native-fp32 vs torch-fp32 comparison on the same
  GPU (`stirling/modal/eval_video.py::metrics`, separate CUDA contexts) agrees
  to three decimals — the `.bin`→`.pt` export is exact. The policy is simply
  numerically fragile across precisions (marginally stable).
- **Fix / decision:** training and native eval default to **fp32**
  (`stirling/modal/train_drone.py`, `--bf16` opts out). An fp32-trained 40M
  HOVER hovers in fp32 (ema_dist ~0.10, full episodes) and renders cleanly on
  the Mac. Documented in `stirling/modal/README.md` (Precision) and tech doc
  §5.2; robustness risk logged in tech doc §11.

### Also this session

- `stirling/modal/eval_video.py` — headless native eval → mp4 (Xvfb + llvmpipe
  + ffmpeg) and a native-vs-torch fp32 metrics entrypoint for GPU sanity checks.
- `pufferlib/torch_pufferl.py` — honor `reset_state` so the recurrent hidden
  state persists across eval rollouts (horizon=1) instead of zeroing every step,
  matching the native backend (correctness fix for the torch eval path).

## 2026-07-14 — Modal training framework + classical controller + Stage 3a wrapper

A large increment spanning training infrastructure, the classical control
baseline, and the first Stage 3 env code.

### Training infrastructure: Modal (new primary path)

`stirling/modal/` — a low-friction cloud-GPU training framework. One command
(`modal run stirling/modal/train_drone.py`) builds the native CUDA backend,
trains the PufferLib drone env on a Modal GPU, and returns the checkpoint to
the local machine. No SSH, provisioning, teardown, or manual file copying.

- **Runtime, cached build:** the native backend is compiled on the first GPU
  call (a real GPU is present, so nvcc `-arch=native` and the driver libs
  resolve) and cached to a Modal Volume keyed by a hash of the C/CUDA
  sources; only env-code changes trigger a recompile. ccache is persisted.
- **Checkpoints returned inline:** both the native `.bin` and a losslessly
  converted torch `.pt` are written to `stirling/artifacts/drone/<tag>/` and
  mirrored to a `stirling-drone-checkpoints` Volume.
- **Experiments are `puffer` args** passed via `--extra`, so reward-weight /
  timestep / task sweeps never rebuild.
- **Verified end-to-end:** smoke runs plus a full HOVER run to ~104M steps
  completed via Modal; eval video rendered (`stirling/modal/eval_video.py`).
- Quickstart: `stirling/modal/README.md`.

### Native → torch checkpoint bridge for Mac eval

`stirling/scripts/convert_native_checkpoint.py` maps the native backend's
flat float32 weight dump onto the torch policy `state_dict` (layouts match
one-to-one; torch-only biases are zeroed, an identical function). This lets a
GPU-trained checkpoint be evaluated visually on a Mac with no GPU:

```bash
puffer eval drone --slowly --load-model-path stirling/artifacts/drone/<tag>/<step>.pt
```

`--slowly` selects the PyTorch backend (Raylib render, CPU load). A macOS
libomp segfault on eval was fixed by retargeting `build.sh`'s libomp to
torch's bundled copy so `_C` and torch share a single OpenMP runtime — the
correct fix, not the `KMP_DUPLICATE_LIB_OK` workaround (which papers over two
OpenMP runtimes in one process and eventually crashes).

### Classical formation controller — reconstructed + validated (PR #1, merged)

The missing MATLAB/Octave draft (tech doc §8) was reconstructed as portable
Python in `stirling/controller/` (tracking law, APF safety filter, formation
manager, control-step composition) and validated in MuJoCo against 4 generic
250-class quads behind a PX4-like velocity inner loop. Meets the §8.4
smoke-test targets under full rigid-body physics: reform 1.29 s (< 2.0 s),
min separation 0.49 m (≥ 0.40 m). Theory writeup:
`stirling/docs/classical_controller_design.md`. This resolves the Stage 3
"classical controller source is missing" blocker (see `stage3_plan.md`).

### Stage 3a — velocity-setpoint wrapper in the env (branch, not yet merged)

`ocean/drone/velocity_controller.h` adds the four-layer velocity-setpoint
stack (classical P law → `k_res`-scaled residual hook → saturate → motor
mapping) in front of the native motor interface, gated by a new
`DroneEnv.control_mode` (default 0 = native motor path, byte-identical to
Stage 1). `control_mode=1, k_res=0` is pure classical velocity control.
Verified with `stirling/tests/test_velocity_wrapper.c` (native path runs;
classical hover settles). Lives on branch `stage3a-velocity-wrapper`;
**still to do:** open its PR, and run the "Stage 1 still scores ~836 under
`control_mode=0`" GPU regression — now straightforward via the Modal path.

### Housekeeping (PR #2, merged) + repo hygiene

macOS eval workflow and Stage 1 automation landed; June project docs
converted from `.docx` to markdown; render videos untracked (regenerable).

### Status after this increment

- **Stage 1 (HOVER baseline):** complete; now retrainable in one command via
  Modal, evaluable on a Mac.
- **Stage 2 (platform recalibration):** still blocked on hardware selection
  (`hardware_platform_shortlist.md`).
- **Stage 3:** classical controller done (a/b groundwork); velocity wrapper
  implemented (3a) pending PR + regression; FORMATION task + observation
  extension (c/d) not started.

### Next steps

1. Open the `stage3a-velocity-wrapper` PR; run its `control_mode=0`
   regression via Modal against the Stage 1 baseline.
2. Port the classical controller from Python into `velocity_controller.h`
   (stage3_plan task b) with the NFR-36 unit-test gate.
3. FORMATION task scaffolding + observation extension (tasks c/d).
