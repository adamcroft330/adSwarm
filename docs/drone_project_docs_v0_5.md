**Formation Racing Drone Swarm — Tech Docs v0.5**

*Project Technical Documentation — Version 0.5*

Updated May 2026 — RL Pipeline Doc Released

**MINOR REVISION: **RL pipeline doc v0.2 published. Simulator choice now closed in favour of extending the PufferLib 4.0 Ocean drone environment. Action space reaffirmed (3-float velocity setpoints) with explicit motor-level escalation path. RL pipeline contents (training stages, observation/action/reward design, env extensions, sim-to-real plan) live in the new RL pipeline document and are referenced from here rather than duplicated. Changes in this revision are confined to Sections 2, 5, 6.2, 11, and 12.

# 1. Project Overview

This document captures the technical decisions, architecture choices, rationale, and known risks for an autonomous formation racing drone swarm. Following the release of competition rules in April 2026, the system is defined as a 4-drone square formation with full onboard compute, navigating an obstacle-rich race course.

## Project Constraints

| **Constraint** | **Value** |
| --- | --- |
| Budget | £10,000 |
| Timeline | ~6 months |
| Team | 5 members, ~7 hrs/week each (~210 total hours) |
| Formation size | 4 drones, square formation |
| Compute location | Onboard only — no base stations permitted |
| Max take-off weight | 1.5 kg per drone |
| Max dimensions | 250 × 250 × 250 mm per drone (with propeller guards) |
| Mandatory hardware | Propeller guards; emergency stop function |
| Sensors permitted | Onboard only — no external motion capture, no base stations |
| Obstacles | Vertical gates, horizontal bars, narrowing corridors, tunnels, elevation changes, partial occlusions |
| Formation flexibility | Shape change permitted; must reform within 2 seconds of any break |
| Course dimensions | Published at least 4 weeks before competition day |

# 2. Core Technical Stack

| **Component** | **Choice** | **Status** |
| --- | --- | --- |
| Simulator | PufferLib 4.0 Ocean drone env, extended for multi-agent FORMATION task | Confirmed (RL pipeline doc v0.2 §2). Supersedes gym-pybullet-drones. |
| gym-pybullet-drones | Held as contingency for sim-to-real fine-tune if Stage 5 transfer fails | Contingency only (RL pipeline doc v0.2 §2.1, option D) |
| Training framework | PufferLib (PuffeRL trainer, PPO via GymnasiumPufferEnv) | Confirmed |
| RL algorithm | PPO with shared policy weights (CTDE) | Confirmed |
| Multi-agent approach | Shared policy across 4 drones | Confirmed |
| Formation architecture | Deformable virtual structure with reconfiguration mode | Confirmed — see §4 |
| Hardware platform | TBD — Crazyflie 2.1 no longer viable | OPEN — see §3 |
| Inference deployment | Onboard (mandated by rules) | LOCKED by competition rules |
| Inter-drone comms | Onboard radio TBD (no ground station relay) | OPEN |
| Localisation | Visual-inertial odometry (VIO) + onboard sensors | Confirmed — see §9 |

# 3. Hardware Platform — Re-Selection Required

**DECISION OPEN: **Carries forward unchanged from v0.4. The Crazyflie 2.1 was selected on the assumption of offboard inference and a non-obstacle course. Both assumptions are now invalid. A new hardware platform must be selected before further development.

## 3.1 Why Crazyflie 2.1 Fails the New Rules

- Onboard compute insufficient: 192 KB RAM cannot host VIO + RL inference simultaneously.

- Companion board infeasible: Crazyflie 2.1 lift margin is too small for a Jetson-class board.

- No external positioning: ArUco-via-mono-camera assumed marker placement and slow flight; obstacle navigation needs richer perception.

- Sensor payload: a forward-facing camera or stereo pair for VIO/obstacle detection exceeds the Crazyflie payload budget.

- The 1.5 kg TOW limit and 250³ mm envelope now permit a much larger drone class — the original size constraint no longer applies.

## 3.2 New Platform Shortlist

| **Class** | **Example** | **Onboard compute** | **Notes** |
| --- | --- | --- | --- |
| 3–5" research quad with companion compute | Holybro X500 v2 + Pixhawk 6C + Jetson Orin Nano | Jetson Orin Nano (40 TOPS, 8 GB) | Standard ArduPilot/PX4 stack; well-supported VIO; mature ecosystem. |
| Compact research drone with integrated compute | Modal AI VOXL 2 / Starling 2 | QRB5165 with onboard VIO | Purpose-built for autonomous research; VIO and obstacle avoidance native; higher unit cost. |
| Custom build on F4/F7 FC + companion SBC | Bespoke 5" frame + Kakute H7 + RPi 5 / Jetson Nano | RPi 5 (8 GB) or Jetson Nano | Highest tuning effort; lowest unit cost; biggest team time risk. |

## 3.3 Selection Criteria

- Compute headroom for PPO inference (~38-input MLP) plus VIO at ≥30 Hz.

- Mature simulator integration — for sim, the chosen autopilot's velocity controller behaviour must be measurable and modellable in our extended PufferLib drone env (RL pipeline doc §2.3).

- Standard autopilot firmware with documented velocity setpoint API (PX4 offboard mode or ArduPilot guided mode are both fine; primary action interface is locked to velocity setpoints — see §6.2).

- Total cost for 4 units + spares within remaining budget.

- Propeller guard availability or feasibility within 250³ mm envelope.

- Hardware emergency stop pathway (RC failsafe + physical kill switch).

- If motor-level escalation is later triggered (RL pipeline doc §2.4), the platform must support a firmware path that exposes motor authority. This is a contingent criterion, not a primary one.

## 3.4 Sunk Cost — Crazyflies Already Procured

The £3,320 already committed against the original BOM is partially recoverable. The Crazyflie 2.1 units retain value as a sim-to-real prototyping fleet for the early single-drone validation stage if a similar control interface is preserved, or can be resold. The Crazyradio 2.0 units are likely sunk. A formal BOM audit should run alongside platform reselection.

# 4. Formation Architecture — Deformable Virtual Structure

Carries forward unchanged from v0.4. The architecture is a three-layer system extended with a deformation/reconfiguration layer to handle obstacle traversal.

## 4.1 Three-Layer Architecture

- Layer 1 — Path Planner (rule-based): Computes the formation centroid path along course waypoints. Also computes obstacle traversal mode per segment (formation-preserving vs single-file vs vertical stack vs reduced-width).

- Layer 2 — Formation Manager (rule-based): Broadcasts target offsets to each drone. The offset set is parameterised by a formation mode (square, line, stack, compressed) selected by the path planner. Each drone receives both the current target offset and the time-to-next-mode-change.

- Layer 3 — Station-Keeping Policy (RL, shared weights): Each drone independently tracks its assigned offset. The policy outputs world-frame velocity setpoints (see §6.2). Detailed policy and training design in RL pipeline doc.

## 4.2 Formation Modes

| **Mode** | **Geometry** | **Use case** |
| --- | --- | --- |
| Square | Default 4-drone square | Open course, gates wider than formation width |
| Line (single-file) | 4 drones along course heading | Narrow corridors, tunnels |
| Stack (vertical) | 4 drones in vertical column | Horizontal bars, low gates |
| Compressed square | Smaller square (scaled offsets) | Marginal-width gates, partial occlusions |
| Diamond | Lead-trail with side wings | Transitional / asymmetric obstacles |

## 4.3 The 2-Second Reform Constraint

Competition rules permit shape change but mandate reformation within 2 seconds of any break. This converts reform speed from an aesthetic concern into a hard timing requirement that must be reflected in (a) the reward function, (b) validation criteria, and (c) the policy's training distribution.

- Reward: large positive shaping reward for re-achieving target offsets within the 2 s window after a mode change or perturbation.

- Penalty: graduated penalty for reformation times exceeding 2 s, scaling steeply beyond the threshold.

- Curriculum: training episodes deliberately inject formation-break events (mode switches, simulated wind gusts, simulated near-misses) to force the policy to learn fast reformation.

- Whether velocity setpoints can satisfy the 2 s reform constraint for every mode transition is one of the conditions that triggers escalation to motor-level control (RL pipeline doc §2.4).

## 4.4 Mode Transition Logic

Mode transitions are computed at path planning time, not learned. Each waypoint segment carries a target formation mode. Transitions are scheduled to begin a configurable lookahead distance before the obstacle, giving the RL policy time to converge on the new offsets within the 2 s budget. The lookahead distance is a tunable hyperparameter.

# 5. Training & Validation Pipeline

**Detailed pipeline lives in the RL pipeline doc. **This section is now a high-level summary; updates to stages and pass criteria belong in the RL doc, not here.

| **Stage** | **Environment** | **Purpose** |
| --- | --- | --- |
| 1. Baseline replication | PufferLib 4.0 drone env (HOVER, native motor-level) | Sanity-check build, training infra, metrics before any modification. |
| 2. Platform recalibration | PufferLib drone env (HOVER, new platform constants) | Confirm env still solves with chosen hardware's mass/inertia/thrust constants. |
| 3. Single-agent station-keeping with formation observations | Extended env with velocity-setpoint wrapper, FORMATION task, stub neighbours | Verify wrapper, observation extensions, and reduced action space all work. |
| 4. Full multi-agent formation in sim | Extended env, num_agents=4, all formation modes, obstacle primitives | Train shared RL policy via PPO; full reward set including 2 s reform shaping; curriculum across modes. |
| 5. Hardware bring-up | Real hardware (autopilot SITL → bench-tied → tethered → free flight) | Single drone first; then 2 drones; then full 4-drone validation. VIO bring-up runs in parallel. |

## 5.1 VIO Bring-Up

Because external motion capture is forbidden and base stations are not allowed, visual-inertial odometry replaces the GPS+IMU EKF for indoor or tunnel sections. VIO bring-up is treated as a discrete sub-project with its own milestones, run in parallel with policy training.

# 6. Policy Architecture

## 6.1 Observation Space

| **Observation** | **Size** | **Notes** |
| --- | --- | --- |
| Own velocity (vx, vy, vz) | 3 floats | Body frame in extended env (RL pipeline doc §2.2) |
| Own angular velocity (p, q, r) | 3 floats | Body frame |
| Orientation quaternion | 4 floats | Full quaternion, hemisphere-handled |
| Target offset, two-scale tanh | 6 floats | Body frame, near and far scales |
| Target normal (body frame) | 3 floats | For orientation-aware tasks |
| Motor RPMs (normalised) | 4 floats | For action smoothness signalling |
| Relative positions of 3 neighbours | 9 floats | Sorted-by-ID for now (see §6.3) |
| Formation mode (one-hot) | 5 floats | Current target mode |
| Time-to-next-mode-change | 1 float | Anticipates upcoming transitions |
| Local obstacle features | TBD | Initial: distance + bearing to next ring/gate |

Total: ~38 floats plus obstacle feature vector. Detailed observation builder design in RL pipeline doc §2.3.

## 6.2 Action Space — Confirmed (carries forward from v0.4)

**DECISION CONFIRMED: **3 floats — vx, vy, vz velocity setpoints in world frame. Yaw remains deterministic from course heading. The onboard autopilot's velocity controller closes the inner loop, identically in sim (via the wrapper added inside the extended PufferLib env, RL pipeline doc §2.3) and on hardware (PX4 offboard mode or ArduPilot guided mode, depending on platform selection).

Motor-level control is held open as an explicit upgrade path. RL pipeline doc §2.4 documents the rationale for the velocity-setpoint choice and the three concrete conditions that would trigger escalation to motor-level (4-float) actions: Stage 4 fails the 2 s reform criterion; published course geometry exceeds what velocity setpoints can credibly handle; or Stage 5 single-drone bring-up reveals tracking lag beyond what sim predicted.

## 6.3 Two MARL Blockers — Status Update

- Neighbour encoding: still requires permutation-invariant approach (attention or pooling); fixed ID ordering remains unsuitable in principle. With only 3 neighbours, sorted ordering is the default in RL pipeline doc; revisit if Stage 4 shows asymmetric failures.

- Heading-relative slot offsets: still required, and now interacts with formation mode. Each (mode, slot index) pair must produce a heading-relative offset vector. Implemented as a lookup table inside the extended env's task layer (RL pipeline doc §2.3).

# 7. Onboard Inference — LOCKED by Competition Rules

With ground stations prohibited, all inference runs on the chosen hardware platform. This eliminates the latency/reliability tradeoff that previously dominated the architecture decision.

| **Constraint** | **Implication** |
| --- | --- |
| No base station / GS | All policy inference, VIO, perception, and formation logic onboard |
| TOW ≤ 1.5 kg | Permits Jetson Orin Nano or QRB5165-class compute |
| ≤ 250³ mm | Tight for stereo cameras + companion compute; mechanical design needs care |
| Inter-drone awareness | Either short-range mesh radio between drones, or fully visual neighbour detection |

**OPEN: **Inter-drone state sharing — mesh radio between drones (e.g. ESPNOW, LoRa, or Wi-Fi mesh) vs purely visual neighbour detection. Decision deferred until VIO/perception bring-up clarifies compute headroom.

# 8. Reward Function

Detailed weights, shaping decisions, and ablation plans live in RL pipeline doc §2.3 and §6. High-level structure unchanged from v0.4:

| **Component** | **Type** | **Description** |
| --- | --- | --- |
| Formation tightness | Penalty | Penalise offset error from current-mode target |
| Course progress | Reward | Reward forward centroid motion along waypoints |
| Inter-drone collision | Penalty (large) | Strong penalty for unsafe separation |
| Boundary / obstacle violation | Penalty (large) | Penalise contact with course boundary or any obstacle |
| Reform-within-2s | Shaping reward | Reward fast convergence after any break or mode switch |
| Slow reformation | Penalty (graduated) | Steep penalty beyond 2 s reform window |
| Smooth setpoints | Light penalty on jerk | Discourage chattering input to the autopilot. Less load-bearing than under direct motor control because the autopilot's velocity controller filters short-timescale noise. |

# 9. Localisation

Carries forward unchanged from v0.4. Centred on visual-inertial odometry.

## 9.1 Absolute Positioning

- Visual-Inertial Odometry (VIO) as primary — onboard stereo or mono-IMU fusion.

- GPS retained as a fallback / outdoor-segment aid where signal is reliable.

- Drift management: course is short (4-week-known dimensions); VIO drift over a single race should be manageable with periodic visual landmark anchoring (e.g. obstacle corner detections).

## 9.2 Relative Localisation (Inter-Drone)

- Option A — visual neighbour detection: each drone detects others via onboard camera (markers or learned detector). Robust to comms loss, but heavy on compute and breaks under occlusion.

- Option B — short-range mesh radio: drones broadcast pose estimates at high rate. Lighter, but adds a comms reliability concern.

- Option C — hybrid: radio broadcast as primary, visual cross-check as sanity / failsafe. Recommended default once perception compute headroom is characterised.

# 10. Safety

Competition rules mandate an emergency stop function. This is a regulatory item, not just a software safeguard.

- Hardware emergency stop: physical RC kill switch on a dedicated channel — mandatory.

- Software watchdog: per-drone watchdog terminates flight on loss of inter-drone updates, VIO failure, or mode-manager failure.

- Propeller guards: mandatory under rules; must be included in airframe design from day one (affects aerodynamics in sim — RL pipeline doc §5.2).

- Failsafe behaviour: on emergency stop, all 4 drones execute coordinated descent or immediate motor cut depending on context — final policy TBD.

# 11. Open Questions & Next Steps

| **Question** | **Priority** | **Triggered by / Status** |
| --- | --- | --- |
| Hardware platform selection (§3) | Critical — blocks everything | New rules |
| Inter-drone comms approach (mesh radio vs visual) | High | No GS allowed |
| VIO stack selection (open-source vs onboard SDK) | High | No external positioning |
| Obstacle perception design (depth, free-space, learned) | High | Obstacle navigation required |
| Formation mode set finalisation (§4.2) | Medium | Course reveal in 4 weeks pre-race |
| Reform-window reward shaping calibration | Medium | Tracked in RL pipeline doc §6.2; sweep at start of Stage 3 |
| Permutation-invariant neighbour encoding (3 neighbours) | Medium | Tracked in RL pipeline doc §4.2; default sorted-ID, revisit if Stage 4 shows asymmetric failures |
| Heading-relative slot offsets per mode | Medium | Implementation tracked in RL pipeline doc §2.3 (task layer) |
| BOM audit — Crazyflie sunk cost / resale | Medium | Platform change |
| Whether to escalate to motor-level action space | Conditional | Triggers documented in RL pipeline doc §2.4; decided at Stage 4 / Stage 5 / course publication |
| Imitation learning fallback (PID demonstrator) | Low — contingency | Carries over |

## 11.1 Items Closed Since v0.4

- Simulator choice: PufferLib 4.0 Ocean drone env, extended. Closed by RL pipeline doc v0.2.

- Action interface (sim-side): velocity-setpoint wrapper inside the extended env, calibrated against the chosen autopilot. Closed by RL pipeline doc v0.2 §2.3 / §2.4.

- Training stage breakdown: 5 stages (baseline / platform recalibration / single-agent + observations / multi-agent + obstacles / hardware bring-up). Closed by RL pipeline doc v0.2 §3.

# 12. Reference Materials

| **Resource** | **URL / Reference** |
| --- | --- |
| RL Pipeline & Environment Plan (companion doc) | rl_pipeline_doc v0.2 — environment extensions, training stages, action-space rationale |
| PufferLib | github.com/PufferAI/PufferLib (4.0 branch) |
| PufferLib drone env (origin) | github.com/tensaur/drone — Sam Turner & Finlay Sanders, MIT-licensed |
| gym-pybullet-drones (contingency) | github.com/utiasDSL/gym-pybullet-drones |
| gym-pybullet-drones paper | arXiv:2103.02142 — Panerati et al., IROS 2021 |
| AttentionSwarm — RL on Crazyflie swarm | arXiv:2503.07376 (2025) |
| Forest swarm (onboard VIO) | Science Robotics 2023, DOI 10.1126/scirobotics.abm5954 |
| Virtual structure formation review | Drones 2024, DOI 10.3390/drones8070320 |
| Domain randomization theory | arXiv:2110.03239 |
| Champion-level drone racing (onboard, motor-level) | Nature 2023, DOI 10.1038/s41586-023-06419-4 |
| VOXL 2 / Modal AI | modalai.com/voxl2 |
| PX4 Offboard mode | docs.px4.io/main/en/flight_modes/offboard.html |
| ArduPilot Guided mode | ardupilot.org/copter/docs/ac2_guidedmode.html |

# Appendix A — Changelog

***v0.5 (May 2026): ****Minor revision following RL pipeline doc v0.2 release. Simulator decision closed (PufferLib 4.0 Ocean drone env, extended; gym-pybullet-drones moved to contingency). §6.2 Action Space clarified to point at the velocity-setpoint wrapper inside the extended env, with motor-level escalation path explicitly tracked. Training pipeline summary in §5 condensed; detailed stages live in the RL pipeline doc. References list updated.*

***v0.4 (April 2026): ****Major revision following competition rules release. Crazyflie 2.1 platform invalidated; onboard inference locked; formation reduced to 4 drones; deformable virtual structure with 5 formation modes introduced; 2 s reform constraint added to reward shaping; VIO replaces GPS+ArUco as primary localisation; obstacle navigation requirements added.*

***v0.3 (earlier April 2026): ****Locked offboard inference, 2× Crazyradio 2.0 with inline mode, training pipeline gym-pybullet-drones → CF SITL → single CF → swarm, observation/action space, ArUco+GPS/IMU EKF localisation.*

***v0.2 (earlier April 2026): ****Initial Crazyflie 2.1 hardware decision, virtual structure architecture, motor constant randomisation rationale, 8-drone formation, training pipeline draft.*
