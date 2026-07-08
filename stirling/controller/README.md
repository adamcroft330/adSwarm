# Classical Formation Controller (Python draft + MuJoCo validation)

Reconstruction of the missing MATLAB/Octave classical-controller draft
described in tech doc v0.6 §8 — the Stage 3 blocker flagged in
`stirling/docs/stage3_plan.md`. The original `.m` files were never
committed anywhere; these modules rebuild the control law directly from
the spec and re-validate it against the recorded smoke-test numbers, in
full rigid-body MuJoCo physics rather than the draft's ideal velocity
dynamics.

File-for-file correspondence with the MATLAB draft (§8.1):

| Module | MATLAB counterpart | Role |
| --- | --- | --- |
| `default_params.py` | `default_params.m` | All gains, limits, safety margins |
| `formation_tracking.py` | `formation_tracking.m` | `u = Kff·v_t + Kp·e + Ki·∫e`, anti-windup |
| `safety_filter.py` | `safety_filter.m` | APF: inter-drone + obstacle + boundary, closing-rate damping |
| `classical_control_step.py` | `classical_control_step.m` | `safety_filter(u_classic + k_res·dv)`, saturate |
| `formation_manager.py` | `formation_manager.m` | Slot offsets ×5 modes, `Rz(yaw)`, linear blend |
| `demo_formation.py` | `demo_formation.m` | Closed-loop 4-drone harness + metrics |

MuJoCo-specific additions (no MATLAB counterpart):

- `quad_autopilot.py` — PX4-offboard-like velocity inner loop
  (velocity P → tilt-limited attitude → SO(3) PD → motor mixing), the
  layer the classical stack's velocity setpoints feed on hardware.
- `drone_model.py` — MJCF builder for 4 generic 250-class quads
  (0.9 kg, 250 mm motor diagonal, TWR ≈ 3.6 — placeholder values until
  Stage 2 platform selection).
- `view_formation.py` — interactive viewer.

## Run it

Headless validation (exits non-zero on target violation):

```bash
python3 stirling/controller/demo_formation.py
```

With the mp4 render + metrics plot (written to
`stirling/artifacts/controller/`):

```bash
python3 stirling/controller/demo_formation.py --video
```

Interactive, real-time, looping (macOS needs `mjpython`, which ships
with the `mujoco` pip package):

```bash
mjpython stirling/controller/view_formation.py
```

Scenario: settle into box → centroid moves off at 1 m/s → tour through
line → box → stack → box → compressed → box → diamond → box → 2.5 m/s
lateral disturbance kick on drone 1 → reform. Translucent spheres are
the live slot targets.

## Validation results (2026-07-07, MuJoCo 3.2.5, 500 Hz physics / 100 Hz control)

| Metric | Result | Target (§8.4) | MATLAB draft ref |
| --- | ---: | ---: | ---: |
| Reform after disturbance kick | **1.29 s** | < 2.0 s | 0.60 s |
| Min inter-drone separation | **0.49 m** | ≥ 0.40 m | 0.54 m |
| Worst formation error post-kick | 0.76 m | TBC | 0.49 m |
| Slowest mode-transition settle | 1.68 s (→stack) | < 2.0 s | — |

The MATLAB reference numbers came from ideal velocity dynamics; these
are through full quad dynamics + a velocity inner loop, so slower
reform and a larger transient are expected. Both hard targets pass.

Tuning notes (deviations from / additions to the spec):

- APF activation distance `d_act = 0.70 m` — set just below the
  tightest steady-state formation spacing (compressed 0.72 m, stack
  0.75 m) so the filter never fights a held formation. Closing-rate
  damping still brakes fast approaches from further out.
- Tuning followed the §8.5 order: inner velocity loop (`kv = 5`,
  τ ≈ 0.2 s) first, then the spec's `Kp = 2.0` outer loop (τ ≈ 0.5 s),
  then safety-filter params. No residual (`dv = None`) anywhere — this
  is the Stage 3a pure-classical baseline mode.

## Relation to Stage 3

This is the import blueprint for the C port (§8.6, stage3_plan.md task
b): everything in the six core modules is numpy-array arithmetic with
no solver dependency and maps directly to C in `dronelib.h` /
`velocity_controller.h`. The NFR-36 unit-test gate (reform ≤ 2.0 s, min
sep ≥ 0.40 m) is what `demo_formation.py` asserts.
