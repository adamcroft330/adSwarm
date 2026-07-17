# Classical Formation Controller — Design & Theory

How the Stage-3 classical station-keeping controller works, why each
piece exists, and how it maps to the eventual C port. Companion to
`stirling/controller/README.md` (which is the inventory + how-to-run +
validation results). The control law itself is specified in
`drone_project_docs_v0_6.md` §8 and the formation architecture in §4;
this document explains the *implementation* in `stirling/controller/`.

Audience: anyone picking up the controller cold — to extend it, tune
it, port it to C, or replace the generic drone constants with a real
platform's numbers.

---

## 1. What the controller is for

Four drones fly as a **box formation** along a moving course. Each drone
must hold its assigned corner of the box, follow the box as it
translates and rotates, briefly morph into other shapes to clear
obstacles, and snap back to box within **2 seconds** after any
disturbance — without ever colliding (≥ 0.40 m separation floor).

The controller is the **Layer-3 station-keeper**: given where a drone
*should* be (its slot target) and where it *is*, it produces a
**world-frame velocity setpoint** — "fly this fast in this direction."
That setpoint is what a real autopilot (PX4 offboard / ArduPilot guided)
consumes, and what the MuJoCo inner loop consumes in simulation.

It is deliberately a **classical** controller (fixed control law, no
learning). In the full Stage-3b system an RL policy adds a small
learned correction on top (§6), but the classical law alone must
already satisfy the competition constraints — it's the safety-critical
baseline and the runtime fallback if the policy misbehaves.

---

## 2. The layered stack

Signal flow for one drone, one control tick (100 Hz):

```
                 course waypoints, obstacle plan
                              │
        ┌─────────────────────▼─────────────────────┐
 L1/L2  │  Formation manager  (formation_manager.py) │
        │  centroid path + mode → slot target        │
        │  target = centroid.p + Rz(yaw)·offset      │
        └──────────────┬───────────────┬─────────────┘
              p_target  │      v_target │  (position + feedforward vel)
                        ▼               ▼
        ┌───────────────────────────────────────────┐
 L3a    │  Nominal tracking (formation_tracking.py)  │
        │  u_classic = Kff·v_target + Kp·e + Ki·∫e   │
        └──────────────────────┬────────────────────┘
                       u_classic│
                        ┌───────▼───────┐
 L3b    (optional)      │  + k_res · dv │◄── RL residual (dv=None ⇒ skip)
                        └───────┬───────┘
                        u_total │
        ┌───────────────────────▼────────────────────┐
 L3c    │  Safety filter, APF  (safety_filter.py)     │
        │  add repulsion (drones, obstacles, bounds), │
        │  then clip to v_max                         │
        └───────────────────────┬────────────────────┘
                          v_cmd  │   world-frame velocity setpoint
        ════════════════════════▼════════════════════  ← hardware boundary
        ┌────────────────────────────────────────────┐
 INNER  │  Velocity autopilot  (quad_autopilot.py)    │  PX4/ArduPilot on
        │  vel P → tilt-limited attitude → SO(3) PD → │  hardware; explicit
        │  motor mixing                               │  model in sim
        └───────────────────────┬────────────────────┘
                          4 motor thrusts → rigid-body dynamics
```

`classical_control_step.py` is the composition of L3a→L3b→L3c. Layers 1
and 2 (the formation manager) sit above it; the inner loop sits below
the hardware boundary and is **not** part of the classical controller —
it's the autopilot the controller talks to. That boundary matters a lot
for robustness (§8).

---

## 3. Layers 1–2 — Formation manager (`formation_manager.py`)

Turns "the formation should be *here*, in *this* shape, heading *this*
way" into a concrete world-frame target for each drone.

### The virtual centroid

The formation is a **deformable virtual structure**: a single virtual
point (`Centroid`) with a position `p`, velocity `v`, yaw (course
heading), and yaw-rate. The four drones are rigidly attached to slots
around it. Move/rotate the centroid and every target moves/rotates with
it. A rule-based path planner (Stage 4 scope) drives the centroid along
the course; for validation we drive it programmatically.

### Slot offsets and heading rotation

Each drone owns a slot index (0–3). Its world target is:

```
target = centroid.p + Rz(centroid.yaw) · slot_offset(mode, slot)
```

`slot_offset` is a lookup table (`slot_offsets()`) giving the shape in
the formation's own frame; `Rz(yaw)` rotates that shape to the course
heading. The five shapes:

| Mode | Geometry | Role |
|---|---|---|
| **box** | 1.2 m square | HOME — default at all times |
| line | single file, 1.0 m spacing | narrow corridors |
| stack | vertical column, 0.75 m spacing | low gates / horizontal bars |
| compressed | 0.6× box (0.72 m square) | marginal-width gates |
| diamond | lead/trail + side wings | asymmetric obstacles |

Slot order (front-left, front-right, rear-right, rear-left) is chosen so
box↔deviation transitions don't make drones cross paths.

### Smooth transitions (the blend)

Snapping the target from box to line instantly would demand an
impossible step in velocity. Instead, on a mode change the offset is
**linearly blended** from the old shape to the new one over
`blend_time` (1.0 s):

```
offset(t) = (1−α)·offset_old + α·offset_new,   α = clamp((t − t_change)/blend_time, 0, 1)
```

The target *glides* between shapes, which is what lets the tracker
converge inside the 2 s reform budget instead of chasing a
discontinuity.

### Feedforward velocity

The manager also outputs each slot's **target velocity** `v_target`, not
just position. It has three parts:

1. the centroid's own velocity (the formation is translating),
2. the blend rate `(offset_new − offset_old)/blend_time` during a
   transition (the shape is morphing),
3. `ω × (Rz·offset)` — the tangential velocity from the formation
   yawing (rotating the box spins the corners).

Feeding this forward (next section) is what stops the drones lagging
behind a moving/turning/morphing formation.

---

## 4. Layer 3a — Nominal tracking law (`formation_tracking.py`)

The core control law. For each drone, with position error
`e = p_target − p`:

```
u_classic = Kff·v_target + Kp·e + Ki·∫e
```

Three terms, three jobs:

- **Kp·e (proportional, Kp = 2.0)** — the workhorse. Commands velocity
  proportional to how far off target the drone is. With ideal velocity
  tracking this gives a first-order response with time constant
  `τ ≈ 1/Kp ≈ 0.5 s` — a disturbance decays to ~2 % in ~2 s, which is
  where the 2 s reform rule comes from. Bigger Kp = faster but eventually
  oscillatory once the inner loop's own lag matters.

- **Kff·v_target (feedforward, Kff = 1.0)** — with only Kp, a drone
  chasing a moving target always lags by `v_target/Kp` (it needs a
  standing error to generate the matching velocity). Feeding the target
  velocity forward cancels that lag directly, so the drone rides *with*
  the formation and Kp only has to correct residual error.

- **Ki·∫e (integral, Ki = 0.3)** — kills steady-state offset from
  persistent disturbances (e.g. a steady wind, or the APF constantly
  nudging a drone). Deliberately *light* — integral action adds phase
  lag and can wind up, so it's the smallest term.

### Anti-windup

Integral terms misbehave when the command saturates: error keeps
accumulating in `∫e` while the output is already maxed, so when the
drone finally catches up the built-up integral overshoots hugely. Two
guards:

- **Conditional integration** — only add to `∫e` while the *unsaturated*
  command `u_raw` is below `v_max`. If we're already saturated, freeze
  the integrator.
- **Clamp** — `∫e` is hard-limited to `±i_limit` (0.5 m·s) per axis
  regardless.

The output `u = clip_norm(u_raw, v_max)` saturates the *vector*
magnitude (not per-axis) so saturation never changes the commanded
*direction* — a corner-case that per-axis clipping gets wrong.

---

## 5. Layer 3c — Safety filter, APF (`safety_filter.py`)

Wraps the (possibly RL-corrected) command and edits it to prevent
collisions. Uses an **Artificial Potential Field**: nearby hazards push
the velocity setpoint away, like the drone sitting in a repulsive force
field. Chosen over the CBF-QP alternative because it needs **no solver**
— pure arithmetic that ports directly to C (§8.6 of the spec).

### Repulsion ramp

For a hazard at distance `d`, repulsion speed:

```
ramp(d) = v_rep_max · (d_act − d) / (d_act − d_floor)
```

Zero at the activation distance `d_act` (0.70 m), rising to `v_rep_max`
(2.0 m/s) at the floor `d_floor` (0.40 m), and continuing to grow below
the floor so violations get shoved out hard. Applied along the unit
vector *away* from the hazard.

### Three hazard types

- **Inter-drone** — repulsion from every other drone within `d_act`,
  **plus a closing-rate damping term**: `k_damp · max(closing_speed, 0)`
  where `closing_speed` is the component of relative velocity along the
  line between them. This brakes *fast approaches* before distance alone
  would trigger a strong response — the difference between "too close"
  and "about to be too close." Only the approaching component is damped
  (drones separating aren't penalised).
- **Obstacles** — sphere primitives; distance measured to the surface
  (`‖p−center‖ − radius`). Stage-4 obstacle geometry plugs in here.
- **Course boundary** — a per-axis push-back when within `bound_margin`
  (0.5 m) of the arena bounds, so the swarm never flies out.

### The `d_act` tuning decision (important, not in the spec)

`d_act` (0.70 m) is set *just below the tightest steady-state formation
spacing*: compressed square = 0.72 m, stack = 0.75 m. If `d_act` were
larger than a held formation's spacing, the APF would fire *while the
drones sit correctly in formation* and fight the tracker — the safety
layer would sabotage the mission. Keeping `d_act` under the tightest
in-formation spacing means the APF is silent during normal flight and
only wakes up for genuine excursions; the closing-rate damping still
catches fast approaches from further out. This is why `stack_spacing`
and `compressed_scale` in `default_params.py` were nudged to keep their
spacings above `d_act`.

The filter finishes with `clip_norm(u_out, v_max)` so the safety edit
can never itself exceed the speed limit.

---

## 6. Layer 3b — The RL residual hook (`classical_control_step.py`)

```
u_total = u_classic                    if dv is None      (pure classical)
u_total = u_classic + k_res · dv        otherwise          (residual RL)
```

The Stage-3b policy emits a 3-float velocity correction `dv`; `k_res`
(default 1.0) scales it. Crucially the residual is added **before** the
safety filter, so the APF's collision guarantee holds *even if the
policy commands something dangerous*. Passing `dv=None` recovers the
exact pure-classical baseline — this is:

- the **Stage-3a benchmark** the residual policy must beat,
- the **runtime fallback** if the policy faults on hardware.

`classical_control_step` also returns `u_classic` itself, so it can be
appended to the policy's observation (RL pipeline §2.5) — letting the
policy see the baseline it's correcting.

---

## 7. The inner loop (`quad_autopilot.py`) — below the boundary

Not part of the classical controller, but needed to validate it against
*real physics* rather than the MATLAB draft's ideal "the drone is its
velocity command" assumption. This is the layer PX4/ArduPilot provides
on hardware; in sim we model it explicitly. It's a standard cascaded
quad controller:

1. **Velocity → acceleration** — `a_cmd = kv·(v_sp − v)`, then add
   gravity to get the required specific force `f_des = m·(a_cmd + g·ẑ)`.
2. **Tilt limit** — cap the horizontal force relative to vertical so the
   drone never commands an extreme lean (`tilt_max` = 35°).
3. **Force → attitude** — the thrust axis must point along `f_des`;
   combined with the yaw setpoint this defines a desired rotation
   `R_des`.
4. **Attitude PD on SO(3)** — geometric attitude error → body torque
   (`kr`, `kw` gains, stiffer in roll/pitch than yaw).
5. **Mixing** — thrust + 3 torques → 4 motor thrusts via the inverse
   mixer matrix (X-configuration, arms at 45°, alternating rotor spin
   for yaw authority).

`drone_model.py` builds the matching MuJoCo model (4 free-body quads,
prop-guard collision spheres, mocap spheres for the live targets).

---

## 8. MuJoCo validation

The controller is validated closed-loop in MuJoCo against full rigid-body
dynamics — deliberately *not* the MATLAB draft's ideal velocity dynamics,
so the numbers reflect what survives contact with a real actuator stack.
Harness: `stirling/controller/demo_formation.py` (headless metrics +
optional mp4); interactive viewer: `view_formation.py`.

**Setup:** 4 generic 250-class quads (0.9 kg, 250 mm motor diagonal,
TWR ≈ 3.6) behind the §7 velocity inner loop, 500 Hz physics / 100 Hz
control, MuJoCo 3.2.5. Pure classical throughout (`dv = None`) — this is
the Stage 3a benchmark mode, no RL residual.

**Scenario:** settle into box → centroid moves off at 1 m/s → tour every
formation mode (line → box → stack → box → compressed → box → diamond →
box) → a 2.5 m/s lateral disturbance kick on one drone → reform.

**Results (2026-07-07)** vs the tech-doc §8.4 Octave smoke-test targets:

| Metric | Result | Target (§8.4) | MATLAB draft ref |
| --- | ---: | ---: | ---: |
| Reform after disturbance kick | **1.29 s** | < 2.0 s | 0.60 s |
| Min inter-drone separation | **0.49 m** | ≥ 0.40 m | 0.54 m |
| Worst formation error post-kick | 0.76 m | TBC | 0.49 m |
| Slowest mode-transition settle | 1.68 s (→stack) | < 2.0 s | — |

Both hard targets pass. The controller is slower than the MATLAB
reference (1.29 s vs 0.60 s reform) precisely because it runs through
full quad dynamics + a velocity inner loop rather than ideal velocity
integration — that gap is the cost the idealised model hid, and it is
expected. `demo_formation.py` exits non-zero if either hard target is
violated, which is the NFR-36 unit-test gate the C port must reproduce
(§11).

Artifacts land in `stirling/artifacts/controller/`: `demo_formation.mp4`
(render), `demo_metrics.json`, `demo_metrics.png` (error + separation
traces). The mp4 is regenerable and not tracked.

**Caveat — scope of this validation.** It is a single nominal run with
generic constants: no sensor noise, wind, latency, or actuator limits
beyond thrust saturation, and no parameter sweep. It establishes "works
for generic 250-class params in clean sim," not robustness — see §9.

---

## 9. Robustness to other drone parameters

This is the key question for reuse, and the layered split answers it:

**The classical controller (Layers 2–3) is largely platform-agnostic.**
It operates entirely in **velocity-setpoint space** — it never sees the
drone's mass, inertia, or thrust. `Kp`, `Ki`, `Kff`, and all the APF
gains are in metres and m/s, so they transfer to a different airframe
**as long as the inner loop still tracks velocity setpoints with a fast
time constant** (here `1/kv ≈ 0.2 s`, comfortably inside the outer
loop's 0.5 s). That separation is exactly why real autopilots expose a
velocity-setpoint interface.

**The platform dependence is concentrated in the inner loop** —
`QuadParams` (mass, inertia, `thrust_max`, `kv`, attitude gains) in
`default_params.py`. Change the airframe and it's the inner loop that
must be retuned, not the formation law. On real hardware PX4/ArduPilot
*is* that inner loop, tuned per airframe by its own autotune.

**Current numbers are generic placeholders**, not a chosen platform:
0.9 kg, 250 mm motor diagonal, TWR ≈ 3.6, plausible 250-class inertia.
They satisfy the competition envelope (NFR-16/17/18) but are stand-ins
until the hardware decision (tech doc §3) closes and Stage 2 replaces
the `BASE_*` constants with measured values.

**What would break it:** an inner loop too sluggish to hit its velocity
setpoints (low TWR, heavy payload, badly tuned) collapses the timescale
separation and the outer gains would need re-tuning; formations tighter
than `d_act` would make the APF fight the formation (see §5); and very
different scales (much larger drones, much larger formations) would want
the geometric params (`box_side`, `d_act`, `v_max`) rescaled. None of
these are in the tracking *law* — they're inner-loop or geometry
choices.

**Not yet tested:** the validation (§8) is a single nominal run with
generic constants and no sensor noise, wind, latency, or actuator limits
beyond thrust saturation. Robustness *claims* beyond "works for generic
250-class params in clean sim" need a parameter sweep (mass / TWR /
inertia) and domain randomisation — that's Stage-3b / Stage-5 work, not
established here.

### Confirmed in practice: the C port (2026-07-16)

The caveat above — *"as long as the inner loop still tracks velocity
setpoints with a fast time constant"* — turned out to be the binding one,
and it is worth recording how it presented, because the failure did **not**
look like a platform problem.

Ported to the PufferLib env (Crazyflie constants, `BASE_K_MOT = 0.15 s`
motor lag), the identical control law reformed in **6.41 s** against
MuJoCo's 1.29 s. The instinct is to suspect the port. It was not the port:

- The motor lag caps the cascade from the bottom. At 0.15 s the reference
  gains (`kp=2`, `kv=5`, `kr=200`) are **unstable** — they diverge — so the
  cascade must detune to `kp=0.6`, `kv=2`. At `kv=2` the velocity loop's
  time constant is 0.5 s, so a 2.5 m/s disturbance coasts `v·τ ≈ 1.25 m`
  before stopping. No outer-loop tuning recovers that.
- **Faster motors alone did not help either** (3.75 s at `k_mot = 0.02 s`
  with the detuned gains). Motors and gains are *coupled*: the platform sets
  the ceiling on the gains, so moving one without the other changes nothing.
  A one-variable-at-a-time sweep exonerated the wrong variable.
- Given the reference gains **and** a realistic motor lag, the C port
  reproduced this document's §8 numbers to ~3% (1.25 s / 0.487 m / 0.76 m).
  That is what confirmed the port was faithful all along.

Resolution: `BASE_K_MOT` lowered to a realistic 0.05 s (a real Crazyflie 2.1
is ~0.02–0.05 s; this project targets faster 5"-class propulsion), and the
env's cascade raised to the reference gains in §11 — the C and Python
configurations have now converged.

**The transferable lesson for Stage 2:** the platform-agnosticism claimed
above is real but *conditional*, and the condition is not soft. An inner
loop that cannot serve the setpoints does not merely degrade the formation
law — it forces a detune that silently changes what the controller is
capable of. When the real platform's constants land, re-run
`stirling/tests/run_velocity_tests.sh`: its motor-lag witness table shows
the cost of the actuator directly.

---

## 10. Timing

- **Outer (this controller):** 100 Hz (`CTRL_DT = 0.01`), matching the
  env's control rate.
- **Inner loop + physics:** 500 Hz (`SIM_DT = 0.002`), i.e. 5 substeps
  per control tick.

Running the inner loop faster than the outer loop is what gives the
timescale separation the design relies on: the velocity setpoint looks
quasi-static to the attitude loop.

---

## 11. Parameter reference

All in `default_params.py`. Tune in the spec's §8.5 order: (1) inner
velocity loop, (2) `Kp`/`Ki`, (3) safety-filter params, (4) only then
the RL residual.

| Param | Value | Role |
|---|---:|---|
| `kff` | 1.0 | velocity feedforward |
| `kp` | 2.0 | position P (τ ≈ 0.5 s) |
| `ki` | 0.3 | light integral |
| `i_limit` | 0.5 | integrator clamp [m·s] |
| `v_max` | 3.0 | setpoint speed limit [m/s] |
| `d_floor` | 0.40 | hard separation floor [m] |
| `d_act` | 0.70 | APF activation distance [m] |
| `v_rep_max` | 2.0 | max repulsion speed [m/s] |
| `k_damp` | 0.8 | closing-rate damping |
| `bound_margin` | 0.5 | boundary repulsion band [m] |
| `k_res` | 1.0 | residual scale |
| `box_side` | 1.2 | home box side [m] |
| `blend_time` | 1.0 | mode-transition blend [s] |
| `mass` | 0.9 | drone mass [kg] — *placeholder* |
| `kv` | 5.0 | inner velocity gain (τ ≈ 0.2 s) |

---

## 12. Path to the C port (Stage 3, task b)

Everything in the six core modules is array arithmetic with **no solver
dependency** — the deliberate reason APF was chosen over CBF-QP. The map
into the env:

- `formation_tracking` + `safety_filter` + `classical_control_step` →
  `ocean/drone/velocity_controller.h` (or extend `dronelib.h`), called
  from `c_step` where it currently calls `move_drone` with raw motor
  actions.
- `formation_manager` → `set_target_formation()` in `tasks.h`.
- The inner loop (`quad_autopilot`) is **not** ported — on hardware it's
  the real autopilot; inside the env the existing motor dynamics play
  that role.
- **NFR-36 gate:** the C port must reproduce `demo_formation.py`'s pass
  criteria — reform ≤ 2.0 s, min separation ≥ 0.40 m — as a unit test
  before Stage 3 proceeds past 3a.

## 13. References

- `drone_project_docs_v0_6.md` §4 (formation architecture), §8
  (classical controller spec, tuning order, smoke-test targets)
- `rl_pipeline_doc_v0_3.md` §2.3–2.5 (control stack, residual RL)
- `stage3_plan.md` (task breakdown, C-port sequencing)
- `stirling/controller/README.md` (inventory, how-to-run, results)
