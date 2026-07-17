# Progress Log

Reverse-chronological record of significant project increments. Each entry
captures what changed, why, and what it unblocks. Complements the planning
docs (`stage3_plan.md`, `rl_pipeline_doc_v0_3.md`) which describe intent;
this records what actually landed.

---

## 2026-07-17 — Reward extension (task e): velocity measured against the target

Closes the blocker flagged in the entry below. Stage 3 (a)–(e) are done; (f) is
unblocked.

### The fix that mattered: relative velocity

`hover_potential` and `check_hover` scored "on target and settled" using
**absolute** velocity, which is only correct for a static target. A FORMATION
drone must cruise with its centroid, so the metric penalised it for performing
the task — and since `reward` contains `alpha_hover * potential`, a residual
would have been rewarded for slowing down, i.e. for leaving formation. Both now
use `|v - v_target|`.

**This is exact identity for every static-target task.** HOVER, ORBIT, CUBE and
FLAG all set `target->vel = 0`, so `|v - v_target| == |v|` — asserted directly
by a gate rather than argued. Only IDLE/FOLLOW/CONGO shift; they carry the
upstream per-tick `target->vel` convention and are untrained demo tasks. `omega`
is deliberately left absolute: the cascade drives yaw rate to zero (yaw is
damping-only), so drones do not rotate with the formation — the slot geometry
rotates around them.

Effect on the FORMATION potential, pure classical: **0.66**, against HOVER's
0.65 — the two tasks are now scored on comparable footing.

### New terms, all inert by default

| Key | Default | Effect |
| --- | --- | --- |
| `alpha_jerk` | 0.0 | penalty on `\|v_cmd - prev_v_cmd\|`; identically zero on the motor path, where `v_cmd` is never written |
| `alpha_align` | 0.0 | penalty per second unaligned beyond `align_time` — NFR-36's 2 s rule as a graduated penalty |
| `align_dist` | 0.15 | "on slot" tolerance (matches the NFR-36 test threshold) |
| `align_time` | 2.0 | grace window |
| `separation_floor` | 0.0 | inter-drone floor (FR-15); **0 skips the O(n²) check entirely** |
| `separation_terminates` | 0 | whether a breach ends the episode |

`separation_floor = 0` is what HOVER wants: it packs 64 independent drones with
unrelated targets into one env, so they routinely pass close and a breach there
is meaningless — and skipping the check keeps its cost off the Stage 1 path.

The default HOVER reward is **bit-identical** to Stage 1: a gate reconstructs
the pre-(e) formula and measures `max |reward - upstream| = 0.00e+00`.

`Log.collisions` was declared, reset and logged upstream but **never
incremented** — every training log so far reported a structural zero. It is now
wired to the separation check, alongside a new `sep_breach` count.

### Alpha re-tune: deferred to the 3b sweep, deliberately

The plan asks to re-evaluate the four alphas against FORMATION. Measured
per-tick contributions, pure classical:

| Term | FORMATION | HOVER |
| --- | ---: | ---: |
| `alpha_hover * potential` | **+0.047** | +0.046 |
| `alpha_dist * d(dist)` | −0.0013 | +0.0018 |
| `alpha_shaping * d(potential)` | +0.00006 | +0.00044 |
| `alpha_omega * \|omega\|` | 0.00025 | 0.00023 |

`alpha_hover * potential` dominates steady state by ~30×; the convergence terms
telescope to ~0 once on-slot, but still drive the 1 m spawn transient, so they
are not dead weight. With the potential now task-correct, the alphas are not
*wrong* for FORMATION — they are untuned, and tuning them is a sweep, not an
analytical exercise. `config/drone.ini` already has `[sweep.env.alpha_*]`
sections for exactly this.

`alpha_jerk`/`alpha_align` are left at 0 rather than guessed: both are latent
under pure classical control (jerk `|dv_cmd|` = 0.0048 m/s; tracking error
0.07 m never leaves `align_dist`, so the timer never runs), so no defensible
size exists until the residual is in the loop.

### Deferred, with reasons

- **Reform-within-2s bonus window** (plan: "200-step exponential bonus after a
  mode-change event") — there are no mode-change events in Stage 3; the
  scheduler is Stage 4, and no disturbance event exists in the env either (the
  NFR-36 test injects its kick by hand). The same 2 s rule is live now as the
  `alpha_align` penalty, so the requirement is represented.
- **Obstacle-contact termination** — no obstacle primitives until Stage 4.

---

## 2026-07-17 — FORMATION spawn fix; (f) is blocked on (e), with the cost measured

Found while setting up (f), the Stage 3a classical benchmark. Two problems, one
fixed, one flagged for a decision.

### Fixed: FORMATION spawned drones nowhere near their slots

`reset_agent` places a drone at a random grid point. That is right for HOVER —
`set_target_hover` then places the *target* near the *drone*. FORMATION inverts
the dependency: the slot is wherever the centroid is, so the drone must move to
it, not the reverse. The random spawn landed **17–58 m** off-slot against a 6 m
oob margin (`hover_target_dist + 1`), so episodes terminated on the first tick.

Measured over 40 s with 4 drones, before the fix:

| | before | after |
| --- | ---: | ---: |
| oob terminations | 1285 | **0** |
| timeouts | 12 | 12 |
| mean tracking error | 1.574 m | **0.064 m** |

The task was ~99% reset churn, and every metric from it was meaningless. Fixed
with `formation_spawn_state()`: the drone starts uniformly within
`FM_SPAWN_DIST` (1.0 m) of its slot and matches the slot velocity, so an
episode begins in formation rather than accelerating into it.

**Why the tests missed it:** the in-env test placed drones on-slot by hand and
ran 800 ticks — under `HORIZON` (1024), so no reset ever fired. A new gate
(`[formation life]`) runs ~4 horizons and asserts spawn offset plus **oob = 0**.

### Resolved by the task (e) entry above — original finding retained below

### Flagged: the reward/metric is HOVER-shaped, and (e) is not done

`check_hover` and `hover_potential` both use **absolute** velocity
(`norm3(agent->state.vel)`), compared against `hover_vel = 0.1`. A FORMATION
drone must cruise with its centroid at ~1 m/s, so the metric penalises it for
performing the task. Measured on a clean run (pure classical, `k_res=0`):

| | value |
| --- | ---: |
| mean tracking error | 0.064 m |
| mean \|v\| (absolute) | 1.031 m/s — the required cruise |
| mean \|v − v_target\| | 0.088 m/s — the real tracking error rate |
| `check_hover`, absolute velocity (today) | 0.778 |
| `check_hover`, velocity relative to target | 0.918 |

**The drone loses 15.2% of its score purely for cruising.** For (f) alone this
is survivable — a floor and a residual measured on the same metric still
compare. But `reward` includes `alpha_hover * curr`, so (g) would train a
residual that is rewarded for *slowing down*, i.e. for leaving the formation.
That is a training pathology, not a scoring quirk, and the plan already
sequences (e) before (f).

The natural fix is to measure velocity (and the potential's velocity term)
relative to `target->vel`. It has a clean property: **HOVER, ORBIT, CUBE and
FLAG all set `target->vel = 0`, so it is exactly identity for them** — no
Stage 1 behaviour change. Only IDLE/FOLLOW/CONGO shift, and those carry the
per-tick `target->vel` units artifact (see the task-c entry) and are untrained
demo tasks. Not taken unilaterally: (e) also covers re-weighting the four
alphas, reform shaping, a jerk penalty, and a richer termination set.

---

## 2026-07-16 — Observation extension 23 → 41 (task d); `num_drones=4` is the FORMATION config

Stage 3 tasks (a)–(d) are now done. Next is (f), the Stage 3a classical
benchmark.

### Obs layout (`dronelib.h`, single source of truth)

`DRONE_OBS_SIZE` is now derived, and `binding.c`'s `OBS_SIZE` and the Python
policy's input dim both follow it (`vec.obs_size` ← `get_obs_size()`), so no
hardcoded width remains anywhere.

| Index | Content |
| --- | --- |
| 0–18 | unchanged upstream block (body vel, omega, quat, two-scale target offset, target normal) |
| 19–27 | 3 neighbour relative positions, body frame, by agent index |
| 28–32 | formation mode one-hot |
| 33 | time to next mode change, tanh-scaled |
| 34–36 | `u_classic`, body frame — the baseline the residual corrects (§2.5) |
| 37–40 | motor RPMs — **still last** (upstream invariant) |

Neighbours needed their own tanh scale (0.5): they live on a 0.4–3 m scale and
the target's coarse 0.1 is tuned for the 30 m grid, which would squash them to
near zero. `u_classic` is stored on `Drone` by `velocity_control_step` and
normalised by the setpoint saturation; it stays zero on the native motor path,
where no classical law runs, which is honest rather than a stub.

**Existing checkpoints no longer load** — the policy's input dim changed 23 →
41. Expected; the Stage 1 baseline was already retired.

### `num_drones` is per-env packing, not throughput — FORMATION needs 4

The plan called for stubbing neighbours as zeros in Stage 3 and wiring real
ones in Stage 4. That turned out to be unnecessary, and chasing it surfaced a
config defect worth recording.

`vecenv.h`'s `my_vec_init` spawns env instances **until `total_agents` is
reached**. So `total_agents = 2048` is the throughput knob; `num_drones` only
sets how many drones share one `DroneEnv` — i.e. one formation, one APF
neighbourhood. Upstream ships `num_drones = 64` because HOVER treats every
drone as an independent episode, so packing is free. For FORMATION it is not:
one env holds one formation, so 64 drones alias 16-to-a-slot. Measured, before
the guard landed:

| `num_drones` | coincident target pairs | min separation over 4 s |
| ---: | ---: | ---: |
| 4 | 0 | 0.583 m |
| 16 | 24 | 0.178 m ✗ |
| 64 | 480 | 0.052 m ✗ |

`num_drones = 4` costs nothing — it yields 512 env instances instead of 32, at
the same 2048 agents. And since the swarm **is** 4 drones (tech doc §2), a
drone has exactly 3 neighbours: every other agent in its env. So the neighbour
block needs no stub — the same code gives zeros at `num_agents = 1` (the
Stage 3a benchmark config) and real neighbours at 4 (Stage 4), with no rewrite.

`c_reset` now **hard-errors** if `task=formation` and `num_drones > 4`, rather
than training on aliased slots. Verified: 1 and 4 reset cleanly, 64 exits 1
with the config fix in the message.

**Consequence for (f)/(g):** the FORMATION runs need `task = 2` and
`num_drones = 4`. The default stays HOVER; these are overrides (below).

### The velocity stack was unreachable from training

Found while working out what a "FORMATION config" is. `pufferl.load_config`
builds **one CLI flag per key in `config/drone.ini`**, then passes the `[env]`
section to `my_init` as kwargs. `control_mode` and `k_res` were never in the
ini — `binding.c` reads them with `dict_get_unsafe` and falls back to defaults,
which is what kept task (a) inert for the Stage 1 regression. The side effect:
no key ⇒ no generated flag ⇒ **no way to select the velocity stack from a
training run at all**. Every Stage 3 control-path run was blocked on this, not
just (f).

Both are now in `[env]` with inert values (`control_mode = 0`, `k_res = 0.0`),
so the default run is unchanged — HOVER on the native motor path — but both are
overridable. No second config file: FORMATION is an invocation.

```
--env.task 2 --env.num-drones 4 --env.control-mode 1
```

`k_res` is written `0.0`, not `0`, deliberately: the flag's type comes from
`ast.literal_eval` of the ini value, so `0` would type it `int` and silently
truncate `--env.k-res 0.5` to `0` — which is exactly the sweep (g) depends on.
Verified end-to-end: defaults resolve to `task=1, control_mode=0, k_res=0.0`,
and the override above resolves to `task=2, num_drones=4, control_mode=1` with
`k_res=0.5` surviving as a float.

### Task ids renumbered: FORMATION is 2

`FORMATION` now sits directly after `HOVER` rather than appended after the
upstream demo tasks — it is this project's task and belongs at the front.
`HOVER` stays at **1**, so the shipped config default is unchanged; `ORBIT`
through `RACE` shift down one (`ORBIT` 2→3 … `RACE` 7→8). Only
`config/drone.ini`'s `task = 1` selects a task by integer anywhere in the repo,
and it still means HOVER; everything else compares enum names, and `get_task()`
resolves by name. A test now round-trips every `TASK_NAMES` entry through
`get_task()` and pins `HOVER == 1` / `FORMATION == 2`, since nothing in the
compiler ties the name table to the enum and a silent remap would be ugly to
debug.

### Tests

One new gate covering the layout: width, RPMs-last, bounded/finite across all
agents, real neighbour geometry at `num_agents=4` vs zeros at 1, one-hot only
under FORMATION, and `u_classic` live under velocity control but zero on the
motor path. Two `_Static_assert`s pin the constants that must mirror across
headers (`OBS_N_FORM_MODES` ↔ `FORM_MODE_N`, `OBS_V_MAX` ↔ `VC_V_MAX`) — the
include direction (`velocity_controller.h` → `dronelib.h`) prevents referencing
them directly.

---

## 2026-07-16 — FORMATION task (task c): formation manager C port + moving-centroid task

Ported `stirling/controller/formation_manager.py` into `ocean/drone/tasks.h`
and wired a `FORMATION` task into the env. Stage 3 tasks (a), (b), (c) are now
done; next is (d), the observation extension.

### What landed

- **`FORMATION` task** added to the `DroneTask` enum (`TASK_NAMES`:
  `"formation"`). `set_target(...)` now takes the env's `Formation*`;
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
