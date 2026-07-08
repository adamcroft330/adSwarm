# Formation Racing Drone Swarm — Tech Docs v0.6

Project Technical Documentation — Version 0.6

Updated June 2026 — Classical controller drafted; sensor suite determined; VIO platform selected; inter-drone ranging architecture decided; formation reframed (box = home).

MINOR REVISION: Following the classical control architecture session (June 2026), this revision closes the sensor suite, selects a primary VIO module, defines the inter-drone ranging strategy, reframes the formation modes (box/square is the home formation; all other modes are transient obstacle-avoidance deviations), and adds a new Classical Controller section recording the MATLAB draft and smoke-test results. The residual RL decision (RL pipeline §2.5) is reaffirmed. Sections updated: 2, 4, 6.2, 7, 8 (new), 9, 11, 12.

# 1. Project Overview

This document captures the technical decisions, architecture choices, rationale, and known risks for an autonomous formation racing drone swarm. The system is defined as a 4-drone box formation with full onboard compute, navigating an obstacle-rich race course (A→B, ≤5 minutes, two official runs, best counts).

## Project Constraints

| Constraint | Value |
| --- | --- |
| Budget | £10,000 |
| Timeline | ~6 months (deadline 20 Nov 2026) |
| Team | 5 members, ~7 hrs/week each (~210 total hours) |
| Formation | 4 drones, box (square) home formation |
| Compute | Onboard only — no base stations permitted |
| Max take-off weight | 1.5 kg per drone |
| Max dimensions | 250 × 250 × 250 mm per drone (with propeller guards) |
| Battery | LiPo 4S maximum |
| Mandatory hardware | Propeller guards; emergency stop function |
| Sensors | Onboard only — no external motion capture, no base stations |
| Obstacles | Vertical gates, horizontal bars, narrowing corridors, tunnels, elevation changes, partial occlusions |
| Formation flexibility | Box is home; transient deviations permitted for obstacle avoidance; must reform within 2 seconds |
| Run format | A→B single traversal, ≤5 min, 2 official runs (best counts) |
| Course dimensions | Published at least 4 weeks before competition day |
| Visual aids | Indoor course with floor tape (black/yellow) — usable as visual navigation anchor |

# 2. Core Technical Stack

| Component | Choice | Status |
| --- | --- | --- |
| Simulator | PufferLib 4.0 Ocean drone env, extended for multi-agent FORMATION task | Confirmed (RL pipeline doc §2) |
| gym-pybullet-drones | Held as contingency for sim-to-real fine-tune if Stage 5 transfer fails | Contingency only |
| Training framework | PufferLib (PuffeRL trainer, PPO) | Confirmed |
| RL algorithm | PPO with shared policy weights (CTDE) | Confirmed |
| Control approach | Residual RL: policy emits 3-float Δv correction added to classical controller output inside c_step | Confirmed (RL pipeline §2.5) |
| Classical controller | P + feedforward + light-I station-keeping; APF safety filter (CBF-QP upgrade path). MATLAB draft complete, smoke-tested. | Confirmed — see §8 |
| Formation architecture | Deformable virtual structure; box = home formation; line/stack/compressed/diamond = transient deviations | Confirmed — see §4 |
| VIO (primary) | Mighty Camera — embedded SLAM module ($60/unit, 10 g, 15 Hz pose + 800 Hz IMU) | Confirmed — preorder batch 2 mid-summer 2026 |
| VIO (backup) | OAK-D Lite stereo camera (£100/unit, ≥30 Hz pose) | Backup — procure now as hedge |
| Inter-drone ranging | Radio broadcast of VIO poses primary (≥20 Hz); UWB ranging modules strongly recommended | Radio confirmed; UWB — see §11 |
| Hardware platform | TBD — Crazyflie 2.1 no longer viable | OPEN — see §3 |
| Inference deployment | Onboard (mandated by rules) | LOCKED |

# 3. Hardware Platform — Re-Selection Required

DECISION: Carries forward unchanged from v0.5. A new hardware platform must be selected before further development.

## 3.1 Why Crazyflie 2.1 Fails the New Rules

Onboard compute insufficient: 192 KB RAM cannot host VIO + RL inference simultaneously.
Companion board infeasible: Crazyflie 2.1 lift margin is too small for a Jetson-class board.
Sensor payload: a forward-facing camera for VIO/obstacle detection exceeds the Crazyflie payload budget.
The 1.5 kg TOW limit and 250³ mm envelope now permit a much larger drone class.

## 3.2 New Platform Shortlist

| Class | Example | Onboard compute | Notes |
| --- | --- | --- | --- |
| 3–5" research quad + companion | Holybro X500 v2 + Pixhawk 6C + Jetson Orin Nano | Jetson Orin Nano (40 TOPS, 8 GB) | Standard ArduPilot/PX4; mature ecosystem. |
| Compact research drone with integrated compute | Modal AI VOXL 2 / Starling 2 | QRB5165 with onboard VIO | Purpose-built; VIO and obstacle avoidance native; higher cost. |
| Custom build | Bespoke 5" frame + F7 FC + RPi 5 / Jetson Nano | RPi 5 (8 GB) or Jetson Nano | Lowest unit cost; highest team-time risk. |

# 4. Formation Architecture — Deformable Virtual Structure

## 4.1 Three-Layer Architecture

Layer 1 — Path Planner (rule-based): Computes the formation centroid path along course waypoints. Selects obstacle traversal mode per segment.
Layer 2 — Formation Manager (rule-based): Computes per-drone target offsets = centroid + Rz(yaw) × slot_offset(mode, slot). Transitions between modes with a linear blend so targets move smoothly and the tracker converges within the 2 s reform window.
Layer 3 — Station-Keeping Controller: Classical controller (§8) + RL residual (§6). Outputs world-frame velocity setpoints consumed by the autopilot inner loop.

## 4.2 Formation Modes — Box is the Home Formation

The box/square formation is the COMPETITION DEFAULT and the formation that must be maintained throughout the race except when an obstacle forces a deviation. Line, stack, compressed, and diamond are TRANSIENT deviations entered only to navigate an obstacle and exited back to box as soon as clearance is achieved. This directly governs the reward structure (§8): the reward anchors on box-formation maintenance; deviation modes earn reward only by enabling course progress.

| Mode | Geometry | Use case | Type |
| --- | --- | --- | --- |
| Square / Box | Default 4-drone square (home) | Open course, standard transit | HOME — default at all times |
| Line (single-file) | 4 drones along heading | Narrow corridors, tunnels | TRANSIENT deviation |
| Stack (vertical) | 4 drones in vertical column | Horizontal bars, low gates | TRANSIENT deviation |
| Compressed square | Smaller square (scaled offsets) | Marginal-width gates, partial occlusions | TRANSIENT deviation |
| Diamond | Lead-trail with side wings | Asymmetric obstacles | TRANSIENT deviation |

## 4.3 Slot Offsets

Each drone is assigned a slot index (1–4). The formation manager computes its world-frame target as:
    target_position = centroid.p + Rz(centroid.yaw) × slot_offset(mode, slot)
The centroid is a virtual point in 3D space with its own position, velocity, and yaw (course heading). Yaw is the orientation of the formation box — how the square is rotated in the horizontal plane. During a mode transition, the slot offset is linearly blended from mode A to mode B so the target moves smoothly. See MATLAB formation_manager.m (§8.1) for the slot offset lookup table and blending implementation.

## 4.4 The 2-Second Reform Constraint

Reward: large positive shaping reward for re-achieving box formation offsets within the 2 s window after any mode change or perturbation.
Penalty: graduated penalty for reformation times exceeding 2 s, scaling steeply beyond the threshold.
Smoke-test result: the classical controller alone (§8) achieves reform in 0.60 s after a disturbance kick — well inside the 2 s budget.
Whether velocity setpoints can satisfy the 2 s reform constraint for every mode transition is one of the escalation triggers to motor-level control (RL pipeline §2.4).

# 5. Training & Validation Pipeline

Detailed pipeline lives in the RL pipeline doc. This is a high-level summary.

| Stage | Environment | Purpose |
| --- | --- | --- |
| 1. Baseline replication | PufferLib 4.0 drone (HOVER, native motor-level) | Sanity-check build, training infra, metrics. |
| 2. Platform recalibration | PufferLib drone (HOVER, new platform constants) | Confirm env still solves with chosen hardware constants. |
| 3. Classical baseline then residual | Extended env with velocity-setpoint wrapper, FORMATION task; first pure classical (dv=0), then RL residual enabled | Establish Stage-3 benchmark with classical controller; then train residual policy on top. |
| 4. Full multi-agent formation in sim | Extended env, num_agents=4, all modes, obstacle primitives | Train shared RL policy; full reward including 2 s reform; curriculum across modes. |
| 5. Hardware bring-up | Real hardware (SITL → bench → tethered → free flight) | Single drone first; then 2-drone; then full 4-drone. VIO bring-up in parallel. |

## 5.1 VIO Bring-Up

VIO bring-up is treated as a discrete sub-project run in parallel with policy training. Target platform: Mighty Camera (primary) or OAK-D Lite (backup). See §9 for full VIO architecture.

# 6. Policy Architecture

## 6.1 Observation Space

| Observation | Size | Notes |
| --- | --- | --- |
| Own velocity (vx, vy, vz) | 3 | Body frame |
| Own angular velocity (p, q, r) | 3 | Body frame |
| Orientation quaternion | 4 | Full quaternion |
| Target offset, two-scale tanh | 6 | Body frame, near and far scales |
| Target normal (body frame) | 3 | For orientation-aware tasks |
| Motor RPMs (normalised) | 4 | From ESC bidirectional DShot telemetry |
| Neighbour relative positions (×3) | 9 | Sorted-by-ID default; attention pooling if Stage 4 shows asymmetry |
| Formation mode (one-hot) | 5 | Current target mode |
| Time-to-next-mode-change | 1 | Anticipates upcoming transitions |
| u_classic (optional) | 3 | Classical controller output — append for policy visibility into baseline |
| Local obstacle features | TBD | Initial: distance + bearing to next ring/gate |

Total: ~38 floats (without u_classic) or ~41 floats (with u_classic appended). The optional u_classic observation gives the policy explicit visibility into the baseline it is correcting.

## 6.2 Control Architecture — Residual RL on Classical Controller

DECISION: Policy output is a 3-float world-frame velocity RESIDUAL (Δvx, Δvy, Δvz). The classical formation controller (§8) runs every control tick and produces u_classic. The residual is added and the safety filter wraps the sum before the output is passed to the autopilot:

    u_classic = formation_controller(target_offset, vel, neighbours)     [runs live every tick]
    u_total   = clip(u_classic + k_res × Δv, −v_max, +v_max)            [RL correction added]
    v_cmd     = safety_filter(u_total, ...)                              [protects −2 contact penalty]

The classical controller plays three mandatory roles: (1) Stage-3 baseline and benchmark the policy must beat, (2) guaranteed-stable fallback (dv=0 recovers pure classical — the safe candidate submission), (3) the prior the residual corrects (warm-start for free; no behaviour-cloning step needed). The classical controller is NOT optional — it is not a contingency. See RL pipeline doc §2.5 for full residual design rationale.

Motor-level control is held open as an explicit upgrade path with three defined triggers: Stage 4 fails the 2 s reform criterion; course geometry requires >5 m/s² lateral acceleration; Stage 5 single-drone bring-up reveals tracking lag beyond sim prediction. See RL pipeline §2.4.

## 6.3 Two MARL Blockers

Permutation-invariant neighbour encoding: sorted-by-ID is the default (3 neighbours makes the cost of getting it wrong small). Revisit attention pooling only if Stage 4 shows asymmetric failures.
Heading-relative slot offsets per mode: implemented in the C task layer as a (mode, slot_index) → offset_vector lookup, rotated into world frame using the centroid heading. The policy sees the resulting target offset; it never directly sees mode metadata beyond the one-hot.

# 7. Onboard Sensor Suite

Every element of the training observation vector must be estimated from onboard sensors on hardware. Estimation error must stay inside the noise envelope randomised during training (NFR-21). The following tiers were determined by mapping each observation field to the sensor that produces it.

## 7.1 Sensor Tiers

### Essential (no flight without these)

| Sensor | Provides | Notes |
| --- | --- | --- |
| IMU — 6-axis (accel + gyro) | Angular velocity (p,q,r) directly; attitude quaternion + velocity via fusion | FC-integrated on all shortlisted platforms |
| Forward camera (global shutter preferred) | VIO (position, velocity, orientation); gate/obstacle perception; floor-tape visual anchoring | Stereo gives metric scale for free. Mighty Camera covers the VIO role; separate forward camera or the same module for obstacle perception. |
| ESC bidirectional DShot RPM telemetry | Normalised motor RPMs — load-bearing obs for action-smoothness signal | Supported by all modern ESCs; enable in autopilot config |
| Inter-drone radio link (Wi-Fi / ESP-NOW) | Neighbour positions and velocities (broadcast of own VIO pose at ≥20 Hz) | No extra hardware if Wi-Fi is available on the companion |

### Strongly Recommended

| Sensor | Provides | Why |
| --- | --- | --- |
| Downward optical-flow sensor | Body-frame horizontal velocity (vx, vy) directly | Body-frame velocity is load-bearing. VIO velocity is noisier and drift-prone indoors. Optical flow + rangefinder is the single biggest improvement to velocity estimation. |
| Downward 1D ToF / lidar rangefinder | Altitude AGL; scale reference for optical flow | Cheap (£20–40), light, pairs with optical flow |
| UWB ranging modules (one per drone) | Direct metric range to each neighbour (~10 cm noise); occlusion-proof | Makes the safety filter's inter-drone avoidance robust to comms dropouts and the partial occlusions specified in the brief. ~£150–250/unit, £600–1000 for 4. |

### Optional / Situational

Barometer — altitude aid; very noisy indoors; FC-integrated and costs nothing to enable.
Magnetometer — yaw aid; unreliable near motors and metal structures; prefer VIO yaw indoors.
GPS — degraded indoors per the brief; retain as outdoor fallback; do not depend on it.

## 7.2 Onboard Inference Compute

| Requirement | Target | Status |
| --- | --- | --- |
| Onboard RAM | ≥4 GB (VIO + RL inference + perception co-resident) | Platform-dependent |
| Compute class | Jetson Orin Nano-class or QRB5165-class | TBC — platform selection |
| Control loop frequency | ≥30 Hz | Provisional (NFR-01) |
| Policy inference latency | <10 ms per step | TBC — platform benchmark |

# 8. Classical Controller — MATLAB Draft

The classical station-keeping controller is the mandatory Layer-3 baseline. It runs live every control cycle on hardware and inside the PufferLib env's c_step, producing u_classic which the RL residual corrects. MATLAB draft complete June 2026; smoke-tested in Octave.

## 8.1 Files

| File | Role |
| --- | --- |
| default_params.m | All gains, limits, safety margins. Tune here first. |
| formation_tracking.m | Nominal law u_classic = P + feedforward + light-I (anti-windup) + yaw. |
| safety_filter.m | Avoidance: inter-drone, obstacle, course boundary. APF default (C-portable); CBF-QP upgrade option. |
| classical_control_step.m | One tick: safety_filter(u_classic + k_res×dv) then saturate. Pass dv=[] for pure classical. |
| formation_manager.m | Layer 2: slot offsets per mode, heading rotation (Rz(yaw)×offset), transition blend. |
| demo_formation.m | Closed-loop 4-drone harness: hold + square↔line + disturbance reform. |

## 8.2 Control Law

Nominal tracking (P + feedforward + light-I with conditional anti-windup):
    u = Kff × v_target + Kp × (p_target − p) + Ki × ∫e dt
Kp ≈ 2.0 gives closed-loop time constant ~0.5 s — reform settling well inside the 2 s rule. Feedforward (Kff = 1.0) prevents lag on a moving formation. The light-I term kills steady-state offset from disturbances.

## 8.3 Safety Filter

APF (default, C-portable): artificial potential field repulsion for inter-drone separation, obstacles, and course boundary. Includes closing-rate damping on the inter-drone term to brake fast approaches. No solver dependency — maps directly to C.
CBF-QP (upgrade): solves a small constrained QP to minimally edit the command while guaranteeing separation. Uses quadprog in MATLAB; falls back to APF automatically when no solver is present. In C, replace with a lightweight active-set QP (handful of constraints). Gives a hard safety guarantee regardless of the RL residual — the safety filter wraps u_classic + k_res×dv, so the guarantee holds even if the policy misbehaves.

## 8.4 Smoke-Test Results (Octave, APF path)

| Metric | Result | Target |
| --- | --- | --- |
| Reform time after disturbance kick | 0.60 s | < 2.0 s (competition rule) |
| Min inter-drone separation | 0.54 m | ≥ 0.40 m floor (FR-15 TBC) |
| Worst formation error | 0.49 m | < TBC m (post-kick transient) |

## 8.5 Tuning Order

Strictly follow this order to avoid masking bugs with gain tuning:
(1) Autopilot/velocity wrapper inner loop — close the velocity-tracking loop first.
(2) Kp/Ki of the classical formation controller — tune against the moving target.
(3) Safety filter parameters (d_sep, v_rep_max, k_damp) — tune with no residual.
(4) Only then enable the RL residual — the baseline must be solid before adding the correction.

## 8.6 C Port (Env + Hardware)

The MATLAB files are the import blueprint. The APF path is dependency-free and maps directly to C; port to dronelib.h or velocity_controller.h inside the env. The CBF path requires a small C QP for the env and hardware path. Runtime on hardware: u_classic and the policy forward pass both run every control cycle on the companion computer. The safety filter wraps both outputs before the autopilot command.

# 9. Localisation

Centred on visual-inertial odometry. External motion capture is forbidden; base stations are not allowed; GPS is degraded indoors. Floor tape (black/yellow) on the course provides opportunistic visual anchoring.

## 9.1 VIO Module — Mighty Camera (Primary)

| Specification | Value |
| --- | --- |
| Type | Embedded SLAM — 6-DoF tracking on a single board |
| Camera | OV9281 global shutter, 160° FOV |
| IMU | ICM-42670-P 6-axis, 800 Hz fusion |
| Visual odometry rate | 15 Hz pose estimates (800 Hz IMU interpolates between) |
| Interface | USB 2.0 (raw camera + IMU + poses) or UART/MAVLink (poses only) |
| Dimensions / mass | 33 × 30 mm, 10 g |
| Power | 0.2 W idle / 0.8 W peak |
| Price | $60/unit — £190 for 4 drones (negligible within £10k budget) |
| Loop closure | Planned / WIP — ETA mid-summer 2026 (batch 2) |
| Availability | Batch 2 mid-summer 2026 — preorder now |

NOTE on 15 Hz pose rate: NFR-04 targets ≥30 Hz VIO update rate; Mighty's visual odometry runs at 15 Hz. The 800 Hz IMU fusion interpolates pose between visual updates, giving a smooth high-rate pose estimate for the control loop. In practice the effective pose rate for control purposes is 800 Hz (IMU-propagated), with visual corrections at 15 Hz. If testing reveals this is insufficient, switch to OAK-D Lite backup.
NOTE on loop closure: Without loop closure, monocular SLAM accumulates scale drift (~1–5% over a traverse). On a short race course, this could be 0.5–2 m cumulative error. If loop closure does not ship with batch 2, calibrate scale using UWB ranging between drones or a known-distance reference run. Monitor actual drift during Stage 5 bench testing.

## 9.2 Backup — OAK-D Lite

| Specification | Value |
| --- | --- |
| Type | Stereo depth camera + IMU |
| Pose rate | ≥30 Hz (meets NFR-04 directly) |
| Scale | Stereo gives metric scale — no monocular drift ambiguity |
| Price | ~£100/unit, £400 for 4 drones |
| Status | Available now — procure as hedge against Mighty batch 2 slipping |

If Mighty batch 2 arrives later than 15 July 2026 (leaves insufficient integration time before competition), switch immediately to OAK-D Lite for Stage 5 hardware bring-up. The observation builder should abstract the pose source so the controller and policy receive the same interface regardless of which module is fitted.

## 9.3 Absolute Positioning

VIO (Mighty Camera or OAK-D Lite) as primary.
GPS retained as a fallback / outdoor-segment aid where signal is reliable.
Floor tape (black/yellow per brief): visible to the forward camera; feature-rich, high-contrast. Use opportunistically as a visual anchoring landmark.

## 9.4 Inter-Drone Relative Localisation

| Method | Reliability | Occlusion | Cost | Recommendation |
| --- | --- | --- | --- | --- |
| Radio broadcast of VIO poses (≥20 Hz) | ~95% indoors | No — RF propagates | Negligible (Wi-Fi on companion) | PRIMARY — implement first |
| UWB ranging modules | ~99% with radio | No — multipath helps in clutter | ~£150–250/unit (£600–1000 total) | STRONGLY RECOMMENDED — add pre-Stage 5 |
| Visual detection (markers/LEDs) | ~70–80% in clutter | YES — complete failure under occlusion | Low (uses existing camera) | Optional sanity-check only; never primary |

Runtime observation builder:
If radio broadcast is fresh (<100 ms old): use broadcast pose directly for neighbour observation.
If UWB available: fuse direct range with last-known broadcast position for cleaner separation estimate.
If broadcast is stale: extrapolate from last-known position + velocity. Mark as stale in obs if RNN variant is used.

# 10. Safety

Hardware emergency stop: physical RC kill switch on a dedicated channel — mandatory (rules).
Software watchdog: per-drone watchdog terminates flight on loss of inter-drone updates, VIO failure, or mode-manager failure.
Propeller guards: mandatory under rules; must be included from day one (affects aerodynamics in sim — RL pipeline §5.2, NFR-23).
Safety filter: the APF/CBF safety filter wraps the combined controller+residual output every cycle, guaranteeing inter-drone separation and obstacle clearance regardless of the RL policy output. This is the primary technical protection against the −2 contact-penalty rule.
Failsafe on zero residual: zeroing the RL residual at runtime recovers the pure classical controller — the known-stable fallback. A runtime switch to dv=0 is the safe mode if the policy misbehaves in flight.

# 11. Open Questions & Next Steps

| Question | Priority | Status |
| --- | --- | --- |
| Hardware platform selection (§3) | Critical — blocks everything | New rules — still open |
| UWB modules — procure and integrate | High | NEW — recommended; decide pre-Stage 5 |
| Mighty Camera batch 2 arrival timing | High | NEW — preorder now; hedge with OAK-D Lite |
| VIO stack integration + drift calibration | High | NEW — begin bench testing when Mighty/OAK-D arrives |
| CBF-QP vs APF safety filter | Medium | NEW — APF is default; promote CBF to default and port to C if scoring analysis warrants the stronger guarantee |
| Inter-drone comms: mesh radio vs ESP-NOW vs Wi-Fi | High | No ground station allowed |
| Obstacle perception design (depth, free-space, learned) | High | Obstacle navigation required |
| Formation mode set finalisation | Medium | Course reveal in 4 weeks pre-race |
| Reform-window reward shaping calibration | Medium | Sweep at start of Stage 3 |
| Permutation-invariant neighbour encoding | Medium | Default sorted-ID; revisit if Stage 4 asymmetric |
| Whether to escalate to motor-level action space | Conditional | Triggers documented in RL pipeline §2.4 |
| BOM audit — Crazyflie sunk cost / resale | Medium | Platform change |

## 11.1 Items Closed Since v0.5

Residual RL decision: classical controller mandatory as baseline/fallback/prior; policy emits 3-float Δv correction. Closed by RL pipeline doc v0.3 §2.5.
Classical controller drafted: MATLAB files complete, smoke-tested (reform 0.60 s, sep 0.54 m). Closed this session — see §8.
Sensor suite: IMU, forward camera, ESC DShot RPM, radio broadcast (essential); optical-flow + ToF, UWB (strongly recommended). Closed this session — see §7.
VIO module: Mighty Camera ($60/unit, 10 g) as primary; OAK-D Lite as backup. Closed this session — see §9.
Formation reframing: box/square is home formation; line/stack/compressed/diamond are transient obstacle-avoidance deviations. Closed this session.
Run format: A→B single traversal, ≤5 min, 2 official runs (best counts). Floor tape is visual nav anchor.

# 12. Reference Materials

| Resource | URL / Reference |
| --- | --- |
| RL Pipeline & Environment Plan v0.3 (companion doc) | rl_pipeline_doc v0.3 — residual RL design (§2.5), env extensions, training stages, action-space rationale |
| Classical Controller (MATLAB draft) | classical_controller/ folder — default_params.m, formation_tracking.m, safety_filter.m, classical_control_step.m, formation_manager.m, demo_formation.m |
| PufferLib | github.com/PufferAI/PufferLib (4.0 branch) |
| PufferLib drone env (origin) | github.com/tensaur/drone — Sam Turner & Finlay Sanders, MIT-licensed |
| Mighty Camera | mightycamera.com — $60/unit embedded SLAM module, 10 g, 15 Hz pose + 800 Hz IMU |
| OAK-D Lite (backup VIO) | luxonis.com/depthai — stereo camera + IMU, ≥30 Hz pose, £100/unit |
| gym-pybullet-drones (contingency) | github.com/utiasDSL/gym-pybullet-drones |
| AttentionSwarm — RL on Crazyflie swarm | arXiv:2503.07376 (2025) |
| Forest swarm (onboard VIO) | Science Robotics 2023, DOI 10.1126/scirobotics.abm5954 |
| Distributed MPC formation control | Drones 2025, 9(5):366 — DMPC velocity-setpoint formation controller; cooperative-capability metrics |
| Champion-level drone racing (Swift) | Nature 2023, DOI 10.1038/s41586-023-06419-4 |
| Batra et al. — end-to-end MARL swarms | CoRL 2022, PMLR 164:576-586 — PPO formation RL; DeepSets neighbour encoding |
| PX4 Offboard mode | docs.px4.io/main/en/flight_modes/offboard.html |
| ArduPilot Guided mode | ardupilot.org/copter/docs/ac2_guidedmode.html |

# Appendix A — Changelog

v0.6 (June 2026): Classical control session. (1) Residual RL decision reaffirmed — classical controller promoted from contingency to mandatory (§6.2). (2) Classical controller MATLAB draft added (§8): P + feedforward + light-I law, APF safety filter with closing-rate damping, CBF-QP upgrade path; smoke-tested reform 0.60 s / min sep 0.54 m. (3) Sensor suite determined (§7): IMU + camera + ESC DShot + radio broadcast essential; optical-flow + ToF + UWB strongly recommended. (4) VIO module selected (§9): Mighty Camera ($60/unit, 10 g, 15 Hz pose + 800 Hz IMU) as primary; OAK-D Lite backup; loop closure caveat noted. (5) Inter-drone relative localisation architecture defined (§9.4): radio broadcast primary, UWB strongly recommended, visual detection optional. (6) Formation reframed (§4.2): box/square = home formation; all other modes = transient obstacle-avoidance deviations. (7) Run format and floor tape anchor recorded in constraints table (§1).

v0.5 (May 2026): Minor revision following RL pipeline doc v0.2 release. Simulator decision closed (PufferLib 4.0 Ocean drone env). §6.2 action space clarified to point at velocity-setpoint wrapper. Training pipeline summary condensed.

v0.4 (April 2026): Major revision following competition rules release. Crazyflie 2.1 platform invalidated; onboard inference locked; formation reduced to 4 drones; deformable virtual structure with 5 formation modes introduced; 2 s reform constraint added; VIO replaces GPS+ArUco; obstacle navigation requirements added.
