**Formation Racing Drone Swarm — RL Pipeline & Environment Plan**

*Project Technical Documentation — RL Pipeline v0.3*

Updated June 2026 — residual RL on classical controller formalised; §2.5 added; companion references updated to project doc v0.6 and requirements doc v0.4.

**Companion to:** drone_project_docs v0.6 (technical decisions) and requirements_doc v0.4 (system requirements). This document covers only the RL pipeline: environment, training stages, observation/action/reward design, and the pieces of the policy architecture that are independent of the (still-open) hardware platform decision. Hardware-dependent items are flagged.

# 1. Purpose & Goals

This document defines the reinforcement learning pipeline for the 4-drone formation racing system, from simulator choice through policy training, validation, and the path to real-hardware deployment. It captures decisions made and questions still open as of June 2026.

## 1.1 What the RL system must deliver

- A shared-weights policy that, given an individual drone's local observations and assigned formation slot, outputs a world-frame velocity RESIDUAL (Δvx, Δvy, Δvz) that is added to the classical controller's output every control tick. The combined command holds formation, transitions between formation modes, and reforms within 2 seconds of any break. Yaw remains deterministic from the centroid heading. The onboard autopilot's velocity controller closes the inner loop in both sim and real flight (see Sections 2.4 and 2.5 for the action-space and residual RL decisions).

- A policy that transfers from simulation to real hardware without retraining, validated on a single drone before scaling to the 4-drone swarm.

- A training pipeline reproducible by any team member from a clean machine, with experiment tracking and reasonable hyperparameter discipline.

- Throughput sufficient to support iterative reward and curriculum design within the project timeline (~6 months remaining at ~210 hours total team effort).

## 1.2 Non-goals (for clarity)

- We are not building a general-purpose drone RL framework. Code outside the formation racing problem will not be maintained beyond the project.

- We are not training a perception stack from raw pixels in this document — visual-inertial odometry and obstacle perception are separate sub-projects (see tech doc Section 9). The RL policy consumes processed state, not images.

- We are not pursuing GPU-accelerated physics or differentiable simulators. PufferLib's CPU-vectorised C environments are fast enough for our problem size.

# 2. Environment Decision

**DECISION:** Extend the PufferLib 4.0 Ocean drone environment (originally by Sam Turner and Finlay Sanders, MIT-licensed, included in PufferLib 4.0). We add a multi-agent FORMATION task, formation-mode logic, obstacle primitives, the sim-side classical controller, and platform-specific physical constants.

## 2.1 Options considered

| **Option** | **Summary** | **Verdict** |
| --- | --- | --- |
| A. gym-pybullet-drones via PufferLib Gymnasium adapter | High-fidelity PyBullet physics, mature ecosystem, used in the original v0.2/v0.3 plan. | Rejected as primary. PyBullet at ~1–10k steps/s per worker is two to three orders of magnitude slower than PufferLib's C envs. Workable if compute were not a constraint, but our team-hours budget makes fast iteration on reward/curriculum the dominant concern. |
| B. Custom C Ocean environment from scratch | Hand-rolled 6-DOF rigid-body quadrotor in C, written against PufferLib's binding template. | Rejected. Reimplements work that already exists at production quality. The PufferLib drone env is exactly the env we would have written, minus the formation-specific extensions. |
| C. Extend PufferLib 4.0 Ocean drone env (chosen) | Use existing single-agent quadrotor as the physics core; add formation task, obstacle primitives, multi-agent reward structure, classical controller, and residual RL wrapper. | Chosen. Physics fidelity, domain randomisation, RK4 integrator, and quaternion observation handling are already correct. The multi-agent scaffolding (multiple Drone instances, nearest-neighbour query, collision check) is already present in dronelib.h though not exercised by current tasks. |
| D. Hybrid: pretrain in extended PufferLib env, fine-tune in pybullet | Cheap pretraining for station-keeping and formation logic, then short pybullet fine-tune for sim-to-real. | Deferred. Re-evaluate after Stage 1 results. If the C env's randomisation envelope is wide enough, the pybullet fine-tune adds work without clear benefit. Keep as a contingency. |

## 2.2 What the existing env already provides

Reading dronelib.h and drone.h on the PufferLib 4.0 branch, the env ships with the following, all of which we can use unchanged or with minor modification:

- Full 6-DOF quadrotor dynamics with quaternion attitude representation, RK4 integration at 500 Hz, and action rate at 100 Hz (5 substeps per action).

- Cross-frame motor torque model (M1=FR, M2=BR, M3=BL, M4=FL) with arm_factor = arm_len / sqrt(2), matching the Crazyflie 2.1 brushless datasheet referenced in the source.

- First-order motor RPM dynamics with time constant k_mot, gravity, propeller drag (k_drag yaw moment), linear body drag (b_drag), angular damping (k_ang_damp), and gyroscopic coupling. All physical parameters are individually domain-randomised at episode init by a single dr factor.

- 23-float observation vector (body-frame velocity, body-frame angular velocity, full quaternion, two-scale tanh-encoded body-frame target offset, body-frame target normal, normalised RPMs).

- Native 4-float action space mapping to motor RPM commands, centred so action=0 yields hover thrust given the randomised mass and thrust coefficient. Note: this is the env's native interface; we wrap it with a sim-side velocity-setpoint controller — see Sections 2.3 and 2.4.

- HOVER and RACE tasks. RACE includes ring waypoints with directional crossing detection and ring-buffer management.

- A multi-agent skeleton: env->agents is an array of Drone, c_step iterates per-agent, observations and actions are per-agent slices. nearest_drone() and check_collision() are present in dronelib.h. The current tasks are functionally single-agent but the data plumbing is already multi-agent.

- Logging structure (Log struct, EMA tracking of distance, velocity, angular velocity) suitable for our metrics out of the box.

## 2.3 What we need to add

The extension surface is well-bounded. The physics core is untouched. Changes are concentrated in the classical controller and residual wrapper around the env's motor interface, the task layer, the observation builder, and the reward calculation in c_step.

### Control stack inside c_step

The env's native action is 4 motor RPMs. The control stack inside c_step now has three layers:

1. **Classical controller** (runs every tick): `u_classic = formation_controller(target_offset, vel, neighbours)` — see Section 2.5 and tech doc §8.

2. **RL residual** (policy output): `u_total = clip(u_classic + k_res × Δv, −v_max, +v_max)` where Δv is the 3-float world-frame correction emitted by the policy.

3. **Safety filter**: `v_cmd = safety_filter(u_total, ...)` — APF or CBF-QP; hard inter-drone separation guarantee regardless of RL residual.

4. **Velocity-to-motor PID** (lowest layer): converts v_cmd to 4 motor RPMs that the existing dronelib.h physics integrates. Tuning target: match the velocity-tracking response of PX4 offboard mode and ArduPilot guided mode within a calibrated envelope. Both autopilots ship documented gain ranges; we use those as priors and randomise within ±15% during training to capture mismatch.

All four layers live in dronelib.h (or velocity_controller.h) below the env API. Passing dv=[] or zeroing Δv recovers the pure classical controller — the known-stable fallback. The motor-level path can be restored later without changing the policy interface above the c_step boundary.

Once the hardware platform is selected (tech doc §3), bench-measure the chosen autopilot's actual velocity step response and adjust the sim PID gains (and randomisation envelope) to bracket what was measured. This is the single most important sim-to-real calibration step.

### Task layer (tasks.h / tasks.c)

- Add a FORMATION task alongside HOVER and RACE. set_target() for FORMATION computes each drone's target as centroid + heading-rotated slot offset for the current formation mode.

- Path planner (rule-based, runs in C alongside the env): advances a centroid waypoint cursor, exposes current formation mode and time-to-next-mode-change.

- Formation modes: square/box (home), line, stack, compressed-square, diamond. Slot offsets are heading-rotated per mode and stored in a small lookup table. See formation_manager.m (tech doc §8.1) for the reference implementation.

### Observation extensions

The current 23-float vector is per-agent and carries no neighbour or formation information. We add the v0.6 multi-agent observations and a placeholder for obstacle features, broken into a fixed-size per-agent vector. Target obs size: 23 + 9 (3 neighbours × 3) + 5 (mode one-hot) + 1 (time-to-transition) + obstacle slot = 38 floats plus obstacle features. Optionally append u_classic (3 floats) to give the policy explicit visibility into the baseline it corrects.

- Three neighbour relative positions, body-frame, sorted by ID for now (permutation-invariant encoding revisit when we have policy training results — see Section 6).

- Formation mode as a 5-element one-hot.

- Time-to-next-mode-change as a single tanh-scaled float.

- Obstacle feature slot (size and content TBD per §5.3 of requirements doc). Initial implementation: distance and bearing to the next ring/gate, body frame, scaled like the existing target offset.

### Reward extensions

- Reuse the existing alpha_dist, alpha_hover, alpha_shaping, alpha_omega scalars but evaluate them against the agent's current target offset (centroid + slot), not a static target.

- Add a reform-within-2s shaping term: when a mode-change event fires, the next 200 environment steps (2 s at 100 Hz) accrue exponential bonus for offset error decreasing toward zero.

- Add graduated penalty for time-since-last-aligned exceeding 2 s after a mode transition, scaling steeply beyond the threshold.

- Light jerk penalty on velocity-setpoint deltas to discourage chattering input to the autopilot. Less load-bearing than under direct motor control because the autopilot's velocity controller filters short-timescale noise on its own.

- Replace the simple oob check with a richer termination set: exiting course bounds, contacting an obstacle, breach of inter-drone separation. Termination reasons go into the Log struct.

### Obstacle primitives

The RACE task already has rings. We add three more primitives implemented as analytical shapes — collision is point-vs-shape, no mesh import:

- Horizontal bar: axis-aligned line segment at a given height, drone fails on contact.

- Narrowing corridor: pair of parallel walls with adjustable gap, no traversal direction enforced.

- Tunnel: capped rectangular volume the centroid must enter and exit through specified faces.

Vertical gates are already covered by the existing ring primitive with appropriate orientation.

### Platform-specific physical constants

**Critical dependency on hardware reselection (tech doc §3).** The existing env hard-codes Crazyflie 2.1 numbers (BASE_MASS=0.027 kg, BASE_IXX=3.85e-6 kg·m², BASE_ARM_LEN=0.0396 m, etc.). These are wrong for any platform we are likely to choose under the new 1.5 kg / 250 mm rules. Action: once the hardware platform is selected, replace BASE_* constants with measured or spec values for the new airframe, and re-run the existing HOVER task as a sanity check before adding any formation work.

## 2.4 Action space — decision and rationale

**DECISION:** Policy output is 3 floats — a world-frame velocity RESIDUAL (Δvx, Δvy, Δvz) added to the classical controller's output every tick. Yaw is deterministic from centroid heading. Motor-level control is held open as an explicit upgrade path under defined conditions (see below). See Section 2.5 for the full residual RL design.

### Why residual RL on a classical controller

- The classical controller provides a guaranteed-stable baseline. Stage 3 runs it with Δv=0 first, benchmarks its performance, then enables the residual. The RL policy only needs to learn the correction — not the full control problem from scratch.

- Sim-to-real risk is meaningfully lower than pure motor-level control. The autopilot's velocity controller is identical in sim and on hardware. Direct motor commands require every parameter of the motor model — k_thrust, k_mot, drag, mass, inertia — to bracket the real airframe within the randomisation envelope. Champion-level drone racing (Nature 2023) achieved this with a serious system-identification effort we have not budgeted.

- Hardware integration is straightforward: PX4 offboard mode and ArduPilot guided mode are mature and documented.

- Action smoothness comes from the autopilot and classical controller, not from reward shaping. Reward-shaped smoothness is brittle.

- Failure modes are recoverable. Zeroing Δv at runtime recovers the pure classical controller — the known-stable fallback and a safe submission candidate if the policy misbehaves in flight.

- Aligns with the v0.6 architecture (centroid path planner → formation manager → classical controller + RL residual → safety filter).

### What we give up

- Performance ceiling. Velocity setpoints cannot express the coupled aggressive manoeuvres a motor-level policy can. Tight obstacle traversal at racing speed and very fast formation reformations may need more authority than the inner loop can supply at its tuned bandwidth.

- Direct alignment with the cited reference papers (Nature 2023, AttentionSwarm). The published swarm/racing RL results we reference all use motor-level control, so we should expect to underperform the literature on raw agility.

- The 2-second reform constraint is harder. A motor-level policy can throw the airframe through a fast reformation by exploiting coupled attitude-thrust dynamics. We need to verify in Stage 4 that velocity setpoints can settle within the 2 s window for every mode transition. Note: the classical controller alone achieves 0.60 s reform in the MATLAB smoke test (tech doc §8.4), so the residual has a strong prior to build on.

### Conditions to escalate to motor-level control

We escalate to a motor-level action space, replacing the sim-side velocity controller, if any of the following holds at the relevant gate:

- Stage 4 fails the 2 s reform criterion on multiple modes after curriculum convergence and hyperparameter sweep.

- Course publication (4 weeks pre-race) reveals geometry tighter or faster than what velocity setpoints can credibly handle. Concrete trigger: any obstacle requiring >5 m/s² lateral acceleration to traverse in formation.

- Stage 5 single-drone bring-up reaches stable hover but the policy lags the autopilot's velocity controller in a way that produces formation drift in flight tests beyond what sim predicted.

If we escalate, the migration is bounded: the velocity-PID layer in the env is removed, the policy output dimension drops from 3 to 4, the action-smoothness reward becomes load-bearing rather than light, and the hardware path requires whichever firmware on the chosen platform exposes motor authority (Betaflight per project-knowledge guide is the obvious candidate). The rest of the pipeline — observations, formation logic, training infrastructure — is preserved.

## 2.5 Residual RL on Classical Controller

**DECISION:** The classical formation controller (tech doc §8) is mandatory — not a contingency. It runs every control tick inside c_step and on hardware. The RL policy emits a 3-float correction Δv that is added to the classical output.

### Control composition

    u_classic = formation_controller(target_offset, vel, neighbours)   [every tick]
    u_total   = clip(u_classic + k_res × Δv, −v_max, +v_max)          [RL residual added]
    v_cmd     = safety_filter(u_total, ...)                            [protects −2 contact penalty]

k_res is a scalar gain (start at 1.0; tune during Stage 3). The safety filter — APF default, CBF-QP upgrade — wraps the combined output every cycle, giving a hard separation guarantee regardless of what the RL residual does.

### Three mandatory roles of the classical controller

1. **Stage 3 baseline and benchmark.** Stage 3 runs the FORMATION task with Δv=0 (pure classical) first. The resulting metrics are the floor the residual policy must beat. If the residual does not improve on the classical benchmark, something is wrong with the policy or reward.

2. **Guaranteed-stable fallback.** Setting Δv=0 at runtime recovers pure classical control instantly — no retraining, no hardware change. This is the safe submission candidate if the policy misbehaves in flight.

3. **Warm-start prior.** The policy inherits the classical controller's already-correct station-keeping behaviour. It only needs to learn the residual correction, not the full control law. No behaviour-cloning step is needed; the classical controller is the implicit prior baked into the env dynamics.

### Implementation

The classical controller is implemented in MATLAB/Octave (tech doc §8.1: default_params.m, formation_tracking.m, safety_filter.m, classical_control_step.m, formation_manager.m). The MATLAB files are the import blueprint for the C port. The APF path is dependency-free and maps directly to C; port to dronelib.h or velocity_controller.h inside the env before Stage 3 (NFR-36). The CBF-QP upgrade requires a small C QP solver but is optional.

### Optional u_classic observation

The policy can optionally receive u_classic as an observation (3 extra floats, total ~41). This gives explicit visibility into the baseline it is correcting, which may accelerate learning. Wire it in from the start and ablate it if it does not help.

# 3. Training Pipeline

Five sequential stages, each with a clear pass criterion. Stages 1–3 are pure simulation; Stage 4 is the first full multi-agent sim; Stage 5 is hardware. Stages map onto the four-stage pipeline in tech doc §5 with one addition: an explicit baseline-replication stage (Stage 1) before any of our own changes touch the env.

| **Stage** | **Goal** | **Env** | **Pass criterion** |
| --- | --- | --- | --- |
| 1. Baseline replication | Train the unmodified PufferLib drone HOVER task to convergence, including its native motor-level action space. Purpose is to confirm our build, training infra, and metrics work before we change anything. This stage runs at the env's native interface — the velocity-setpoint wrapper and classical controller are added in Stage 3. | PufferLib 4.0 drone (HOVER, motor-level), original Crazyflie params | Reproduce the env's reference hover_score within ~10% of the public baseline. |
| 2. Platform recalibration | Replace BASE_* constants with the chosen hardware platform's values. Re-train HOVER to confirm the env still solves with the new dynamics. | PufferLib drone (HOVER), new platform params | HOVER converges; trajectories visually plausible; no NaN losses with motor_constant ±10–15% randomisation. |
| 3. Classical baseline then residual | Add the velocity-setpoint wrapper and C-ported classical controller (Section 2.3, NFR-36). Run Stage 3a: FORMATION task with Δv=0 (pure classical, no RL) to establish the benchmark the policy must beat. Then Stage 3b: enable the residual policy and train. Neighbours stubbed (zeroed). Verifies the wrapper, classical controller port, extended observation space, and residual action interface all work together. | Extended env (classical controller + residual wrapper, FORMATION task, num_agents=1, stub neighbours) | Stage 3a: classical controller holds a programmatically moving target — record as benchmark. Stage 3b: residual policy improves measurably on the classical-only benchmark. |
| 4. Full multi-agent formation in sim | num_agents=4, real neighbour observations, all formation modes, all obstacle primitives, full reward set including 2 s reform shaping. Curriculum: start with square mode only, then add one mode at a time. | Extended env, num_agents=4, FORMATION task with all modes | All four drones complete a representative course in sim, all mode transitions within 2 s, no inter-drone collisions over 100 evaluation episodes. |
| 5. Hardware bring-up | Single-drone sim-to-real on the new platform, then 2-drone, then 4-drone. VIO bring-up runs in parallel as a separate sub-project (see tech doc §5.1). | Real hardware (autopilot SITL → bench-tied → tethered → free flight) | Single drone holds station within TBC m for 60 s; full swarm completes a course at competition spec. |

## 3.1 Curriculum within Stage 4

Stage 4 carries the largest learning risk and benefits from a deliberate curriculum. Proposed schedule, each stage running until the policy plateaus:

- Square mode only, no obstacles, no mode changes. Pure formation hold.

- Square mode, ring obstacles only, no mode changes. Forces collective course-following while in formation.

- Square + line + stack, mode changes scheduled at fixed waypoints, all obstacle types enabled. Trains the 2 s reform behaviour.

- Full mode set including diamond and compressed-square, randomised mode-change schedules, perturbations injected (simulated wind gusts, simulated near-misses). Hardens the policy.

Curriculum stages can be transitioned manually between training runs or driven by a threshold on hover_score / formation_score. We start with manual transitions; auto-transitions only if manual scheduling proves a bottleneck.

## 3.2 PufferLib trainer configuration

Following the PufferLib 4.0 API summary (project knowledge: pufferlib-4-api.md), training uses pufferl.PuffeRL with the standard PPO loop. Config skeleton:

    env_creator = pufferlib.ocean.env_creator('drone')  # extended in our fork

    vecenv = pufferlib.vector.make(env_creator,
        num_envs=N, num_workers=W, batch_size=B,
        backend=pufferlib.vector.Multiprocessing,
        env_kwargs={'num_agents': 4, 'task': 'formation', ...})

    policy  = FormationPolicy(vecenv.driver_env).cuda()
    args    = pufferl.load_config('drone')  # plus our overrides
    trainer = pufferl.PuffeRL(args['train'], vecenv, policy)

Key parameters to tune (start with defaults from the existing drone config, change only with cause):

- args['vec']['num_envs']: total parallel envs. Aim for ≥1024 to keep PPO batch size healthy.

- args['train']['total_timesteps']: budget per stage. Reference: HOVER converges in ~50–100M steps in the original env on commodity GPU.

- args['policy']['hidden_size']: 128 or 256 to start. The action space is small, the obs space is ~40 floats — a deeper net is unlikely to help.

# 4. Policy Architecture

## 4.1 Shared-weights MLP, optionally with RNN

All four agents share one set of policy weights, conditioned at runtime on each agent's own observation vector (CTDE, see tech doc §2). The default architecture is a 2-layer MLP (128–256 hidden) per the PufferLib custom policy template:

    class FormationPolicy(torch.nn.Module):

        def __init__(self, env):
            super().__init__()
            obs  = env.single_observation_space.shape[0]
            acts = env.single_action_space.shape[0]  # 3-float residual Δv (Section 2.4–2.5); 4 if motor-level escalation triggers

            self.net = torch.nn.Sequential(
                pufferlib.pytorch.layer_init(torch.nn.Linear(obs, 128)),
                torch.nn.Tanh(),
                pufferlib.pytorch.layer_init(torch.nn.Linear(128, 128)),
                torch.nn.Tanh(),
            )

            self.action_mean    = torch.nn.Linear(128, acts)
            self.action_log_std = torch.nn.Parameter(torch.zeros(1, acts))
            self.value          = torch.nn.Linear(128, 1)

The action head outputs Gaussian mean + state-independent log-std for continuous control, mirroring the existing PufferLib drone-env policy in pufferlib/ocean/torch.py. RNN (MinGRU per pufferlib.rnn) is reserved as a contingency if Stage 4 shows the policy needs to remember through occlusions or comms dropouts.

## 4.2 Two MARL blockers carried over from v0.6

- Permutation-invariant neighbour encoding. With 3 neighbours, the cost of getting it wrong is small enough that we ship a sorted-by-ID encoding for Stages 3–4 and revisit attention pooling only if training plateaus on an obviously asymmetric failure mode.

- Heading-relative slot offsets per mode. Implemented in the C task layer as a (mode, slot_index) → offset_vector lookup, rotated into world frame using the centroid heading. The policy never sees mode metadata except through the one-hot and the resulting target offset.

# 5. Sim-to-Real & Domain Randomisation

The existing env's domain randomisation envelope (single dr factor scaling all physical constants by ±dr) is structurally correct but the chosen value matters. v0.6 specifies ±10–15% on motor constants and randomised mass/drag at episode start; we apply the same envelope here.

## 5.1 What is randomised

- Mass, inertia (Ixx, Iyy, Izz), arm length: ±10–15%.

- Thrust coefficient k_thrust, drag coefficient k_drag, motor time constant k_mot: ±10–15%.

- Gravity: ±1% (mostly captures atmospheric / model error rather than real variance).

- Initial pose, velocity, angular velocity at episode reset (already in the env via reset_agent and rndf-based pos sampling).

## 5.2 What needs to be added for sim-to-real

- Sensor noise: IMU noise on omega and the implicit velocity/quaternion observations. The current env reads ground truth; we add Gaussian noise plus optional bias on the observation builder.

- Setpoint-tracking latency: a small delay buffer between policy output and the velocity controller, plus randomised gain mismatch on the controller itself, to capture the variable latency between policy and onboard autopilot. Calibrated against bench measurements on the chosen platform's autopilot during Stage 5 prep.

- VIO degradation model: when the obstacle/perception slot is populated from VIO output on hardware, training observations should include drift, dropouts, and bounded latency. Exact model deferred to VIO bring-up (separate sub-project).

- Aerodynamic effect of propeller guards (NFR-23 in requirements doc): either modelled by adjusting b_drag and arm_len during randomisation, or measured on hardware and baked into the BASE_* constants.

# 6. Risks, Kill Criteria, and Open Questions

## 6.1 Risks specific to the RL pipeline

| **Risk** | **Likelihood** | **Mitigation / kill criterion** |
| --- | --- | --- |
| Extended env breaks when we change BASE_* constants for the new platform | Medium | Stage 2 explicitly tests this. If HOVER fails to converge with new constants, escalate to env author / PufferLib Discord before continuing — do not paper over with reward tuning. |
| Classical controller C port introduces bugs vs MATLAB reference | Medium | NFR-36 validation: unit-test the C port against the MATLAB smoke-test results (0.60 s reform, 0.54 m min sep) before Stage 3 begins. |
| Permutation-invariant encoding turns out to matter at 3 neighbours | Low–medium | Sorted-by-ID is the default. If Stage 4 shows policy performance asymmetric across slot assignments after curriculum convergence, swap in attention pooling — small change, well-bounded. |
| 2 s reform shaping is unstable or game-able | Medium | If the policy learns to break formation deliberately to collect reform bonuses, switch from shaping to terminal penalty only. Detect via inspection of Log.collisions and trajectory replays. |
| Sim-to-real gap larger than expected on the new (heavier, larger) platform | Medium | Stage 5 single-drone is the gate. If transfer fails: (a) widen domain randomisation envelope; (b) introduce hybrid path (option D) with pybullet fine-tune; (c) escalate to motor-level control. Decide at the failure, not preemptively. |
| Sim-side velocity PID diverges from the real autopilot's velocity controller | Medium | Bench-measure the chosen autopilot's velocity step response before Stage 5. Tune the sim PID and its randomisation envelope to bracket the measurement. If late-Stage-4 metrics look fine but Stage 5 single-drone tracking is poor, this is the first thing to suspect. |
| Velocity setpoints are insufficient for the published course or the 2 s reform constraint | Medium | Classical controller alone achieves 0.60 s reform (MATLAB smoke test); residual has strong prior. Detect remaining failure at end of Stage 4 (sim) and Stage 5 single-drone (hardware). Triggers the escalation to motor-level control documented in Section 2.4. Migration cost is bounded but consumes ~30–50 hours we don't have slack for, so this is a real schedule risk. |
| Training compute / wall-clock too long for iteration | Low | PufferLib's whole point is this not happening. If we hit it anyway: profile env first, then reduce env step rate (currently 500 Hz physics is generous), then reduce action rate. |
| Hardware platform decision slips and blocks Stage 2 | High | Stages 1 and 3a (classical-only baseline with stub neighbours and original Crazyflie params) can run independently of the platform decision and absorb early-stage time. Real risk is Stages 4–5, where platform constants matter. |

## 6.2 Open questions

| **Question** | **Owner / Resolved by** |
| --- | --- |
| Final formation mode set (5 listed; do we keep diamond?) | Decide after Stage 4 starts — drop modes that consistently cause training instability. |
| Reward weights (alpha_dist, alpha_hover, alpha_shaping, alpha_omega, plus new reform terms) | Sweep at start of Stage 3 using PufferLib Protein / sweep tooling on a fixed seed budget. |
| k_res tuning — residual gain scalar | Start at 1.0; sweep during Stage 3b alongside reward weights. |
| u_classic as observation — include or ablate? | Wire in from start; ablate in Stage 3b if it does not measurably help. |
| Permutation-invariant neighbour encoding — sorted-ID vs attention pooling | Default sorted-ID; revisit only if Stage 4 shows policy is asymmetric across slot assignments. |
| Obstacle perception observation format on hardware | Open in tech doc / requirements doc. Sim-side placeholder is distance+bearing to next ring; replace once hardware-side perception design lands. |
| RNN vs MLP | MLP first. RNN only if policy needs to handle observation dropouts (occlusions, comms loss) — decide during Stage 4. |
| Whether to keep gym-pybullet-drones in the pipeline as a fine-tune step (option D) | Decide at end of Stage 4 based on sim metrics; commit only if Stage 5 single-drone transfer fails. |

## 6.3 Items now closed by this document

- Simulator choice: PufferLib 4.0 Ocean drone env, extended. (Was open in tech doc v0.4; supersedes the gym-pybullet-drones primary choice carried forward from v0.3.)

- Multi-agent training framework: PufferLib PPO with CTDE shared weights. (Confirmed in v0.4; reaffirmed here with concrete API path.)

- Decision to extend rather than fork or rewrite the env. We work on a fork, but the intent is to upstream the FORMATION task if it ends up clean enough — a question to revisit, not a commitment.

- Action space: 3-float world-frame velocity RESIDUAL (Δvx, Δvy, Δvz), added to classical controller output every tick, with deterministic yaw from centroid heading. Motor-level escalation path documented in Section 2.4 with explicit triggers.

- Residual RL on classical controller: classical controller mandatory as baseline/benchmark/fallback/prior; Δv=0 recovers pure classical; safety filter wraps combined output. (June 2026 session — this document §2.5.)

# 7. Milestones

Coarse milestones tied to the five training stages, sized to the ~210-hour total team budget. Hours are wall-clock per workstream; multiple workstreams run in parallel where the team has parallel skills.

| **Milestone** | **Stage** | **Approx team-hours** |
| --- | --- | --- |
| Build env with --local, run unmodified HOVER training to convergence on one machine. | 1 | 10–15 |
| Reproduce baseline metrics; capture them as a regression target. | 1 | 5 |
| Replace BASE_* constants for selected hardware platform; re-run HOVER. | 2 | 10 (after hardware decision) |
| Port classical controller to C (dronelib.h / velocity_controller.h); unit-test against MATLAB smoke-test results. | 3a | 15 |
| Implement velocity-setpoint wrapper inside the env; verify HOVER still solves with policy reduced to 3 residual actions. | 3a | 15 |
| Run Stage 3a: FORMATION task with Δv=0 (pure classical); record benchmark metrics. | 3a | 5 + compute |
| Implement FORMATION task scaffolding (set_target, slot offsets, mode lookup). | 3 | 20 |
| Extend observation builder; wire formation observations end-to-end with stub neighbours. | 3 | 15 |
| Stage 3b: enable RL residual; train to convergence; verify improvement over classical benchmark. | 3b | 10 + compute |
| Multi-agent enable; implement reform shaping and termination logic. | 4 | 20 |
| Implement obstacle primitives (bar, corridor, tunnel); RACE primitive reused for gates. | 4 | 15 |
| Curriculum runs across formation modes. | 4 | 20 + compute |
| VIO and perception observation modelling for sim-to-real. | 4–5 | 15 |
| Bench-measure chosen autopilot's velocity step response; calibrate sim PID and randomisation envelope. | 5 | 10 |
| Single-drone hardware bring-up (depends on hardware Stage 3 in tech doc). | 5 | 30 |
| Full 4-drone hardware validation. | 5 | 30+ |

**Total estimate:** ~230 team-hours of RL pipeline work, ~20 hours over the 210-hour budget. Added items vs v0.2: classical controller C port and Stage 3a benchmark run (~20 hours). The realistic levers if we need to come in under budget: drop diamond formation mode (saves curriculum time in Stage 4), or reduce the curriculum stage count from 4 to 3. Hardware bring-up estimates are the most uncertain numbers in the table; that is where overrun is most likely regardless.

# 8. References

| **Resource** | **Use** |
| --- | --- |
| PufferLib 4.0 Ocean drone env — pufferlib/ocean/drone (dronelib.h, drone.h, tasks.h) | Base environment we extend. Originally by Sam Turner and Finlay Sanders, MIT-licensed (https://github.com/tensaur/drone). |
| PufferLib docs — puffer.ai/docs.html | Build, train, eval CLI; custom env tutorial (Squared, Target templates). |
| Project knowledge: pufferlib-4-api.md | PufferLib 4.0 Python API for vector, pufferl trainer, custom policies. |
| drone_project_docs v0.6 | Parent technical decisions document. Architecture, classical controller (§8), sensor suite, formation reframing, hardware platform reselection. |
| requirements_doc v0.4 | Functional and non-functional requirements driving Stage 4–5 pass criteria. Classical controller requirements NFR-34–37; sensor suite NFR-26–33. |
| Classical Controller (MATLAB draft) | classical_controller/ folder — default_params.m, formation_tracking.m, safety_filter.m, classical_control_step.m, formation_manager.m, demo_formation.m. C port blueprint. |
| betaflight-rl-guide.html (project knowledge) | Reference deployment path (RL → MSP → real FC). Likely informs Stage 5 if we end up with a Betaflight-class FC; currently superseded by the wider hardware shortlist (PX4 / ArduPilot / VOXL). |
| Champion-level drone racing (Nature 2023, DOI 10.1038/s41586-023-06419-4) | Reference for high-speed onboard RL flight; sim-to-real domain randomisation envelope discussion. |
| AttentionSwarm (arXiv:2503.07376) | Reference for shared-policy RL on real drone swarms; relevant if we revisit attention-based neighbour encoding. |

# Appendix A — Changelog

**v0.3 (June 2026): Residual RL on classical controller formalised. (1) §2.5 added — defines the control composition (u_classic + k_res×Δv, safety filter wraps both), the three mandatory roles of the classical controller (Stage 3 benchmark, dv=0 fallback, warm-start prior), and the optional u_classic observation. (2) §2.4 updated — policy output reframed from "velocity setpoints" to "3-float residual Δv"; rationale updated to reflect classical controller as the implicit prior. (3) §2.3 control stack section rewritten — now describes the four-layer hierarchy (classical controller → RL residual → safety filter → velocity-to-motor PID) inside c_step. (4) Stage 3 split into Stage 3a (pure classical, Δv=0, benchmark run) and Stage 3b (residual enabled, train to beat benchmark). (5) Risk table updated — classical controller C port risk added; velocity-setpoints risk updated to note the 0.60 s classical baseline. (6) Open questions updated — k_res tuning and u_classic ablation added. (7) Milestone table updated with C port and Stage 3a benchmark tasks (~5 additional hours). (8) Companion references updated to drone_project_docs v0.6 and requirements_doc v0.4.**

*v0.2 (May 2026): Action-space patch. v0.1 silently inherited the PufferLib drone env's native motor-level action interface, which contradicted v0.4 tech doc Section 6.2. Reverted to v0.4 decision (3-float world-frame velocity setpoints) and added the sim-side velocity wrapper as the first piece of new env code. Added Section 2.4 with full decision rationale and explicit conditions to escalate to motor-level control. Added two related risks. Added wrapper and autopilot-calibration milestones (~25 hours), pushing total estimate from ~200 to ~225 team-hours, ~15 over budget.*

*v0.1 (May 2026): Initial RL pipeline document. Closes simulator choice in favour of extending the PufferLib 4.0 Ocean drone environment. Lays out a 5-stage training pipeline, observation/action/reward extensions, and an estimated team-hours budget. Flags hardware platform reselection as the binding constraint on Stage 2 onward.*
