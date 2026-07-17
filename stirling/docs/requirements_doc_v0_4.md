# Formation Racing Drone Swarm — Requirements v0.4

System Requirements Document — Version 0.4

Updated June 2026 — Classical controller drafted; sensor suite determined; VIO module selected; inter-drone ranging architecture defined; formation reframed; run format clarified.

MINOR REVISION: Following the classical control architecture session (June 2026), this revision: adds NFR-26 through NFR-33 covering the sensor suite and VIO module; adds PR-08 through PR-11 for inter-drone ranging; updates FR-01 to reflect that box/square is the competition home formation; updates §5 (Perception & Localisation) with the Mighty Camera selection and OAK-D Lite backup; closes the sensor suite, VIO module, and formation-reframing items; adds new open items for UWB procurement, Mighty Camera timing risk, and CBF-QP decision. No existing requirement priorities or pass criteria are weakened.

# 1. Purpose & Scope

This document captures the system requirements for an autonomous formation racing drone swarm. With the classical control architecture now drafted and the sensor suite determined, several previously-open items are now closed; remaining open items are flagged.

# 2. Project Context

| Parameter | Value |
| --- | --- |
| Mission | 4 drones in box formation racing an obstacle-rich course (A→B), fully AI-controlled |
| Budget | £10,000 total |
| Timeline | ~6 months (deadline 20 Nov 2026) |
| Team effort | 5 members × 7 hrs/week ≈ 210 hours total |
| Target hardware | TBD — Crazyflie 2.1 invalidated; reselection in progress |
| Battery | LiPo 4S maximum (competition rule) |
| Compute location | Onboard only (mandated) |
| Max take-off weight | 1.5 kg per drone |
| Max dimensions | 250 × 250 × 250 mm including propeller guards |
| Mandatory hardware | Propeller guards; emergency stop; ESC bidirectional DShot |
| External infrastructure | None permitted (no motion capture, no base stations) |
| Course features | Vertical gates, horizontal bars, narrowing corridors, tunnels, elevation changes, partial occlusions |
| Visual aids | Floor tape (black/yellow) — usable as visual navigation anchor |
| Run format | A→B single traversal, ≤5 min, 2 official runs (best counts) |
| Course dimensions | Published ≥4 weeks before competition day |
| Simulator | Extended PufferLib 4.0 Ocean drone env |
| Control approach | Residual RL: u_total = clip(u_classic + k_res×Δv, v_max); safety filter wraps both |
| Classical controller | P + feedforward + light-I; APF safety filter; MATLAB draft complete |
| VIO (primary) | Mighty Camera $60/unit, 15 Hz pose + 800 Hz IMU fusion |
| VIO (backup) | OAK-D Lite, ≥30 Hz, £100/unit |

# 3. Functional Requirements

## 3.1 Formation Control

| ID | Requirement | Priority | Status |
| --- | --- | --- | --- |
| FR-01 | The system shall maintain 4 drones in a box (square) formation as the COMPETITION HOME formation at all times except during obstacle-avoidance deviations. | Must | Confirmed — box is home formation; deviations are transient |
| FR-02 | Each drone shall compute its own control actions onboard using local observations and broadcast neighbour state. | Must | Confirmed (rules) |
| FR-03 | The formation shall transition between defined formation modes (square/box, line, stack, compressed, diamond) to navigate obstacles. All non-box modes are transient deviations; the system shall return to box as soon as obstacle clearance is achieved. | Must | Confirmed |
| FR-04 | The swarm shall reform to the box formation within 2.0 seconds of any break or mode change command. | Must | Confirmed (rules); MATLAB classical baseline achieves 0.60 s |
| FR-05 | Maximum inter-drone position error from the active mode geometry shall not exceed TBC metres during steady flight. | Must | TBC — testing |
| FR-06 | The system shall detect and respond to imminent inter-drone collision within one control cycle via the safety filter. | Must | Confirmed — APF/CBF safety filter |

## 3.2 Race Course Navigation

| ID | Requirement | Priority | Status |
| --- | --- | --- | --- |
| FR-07 | The swarm shall navigate a pre-defined race course via waypoints (A→B, ≤5 min). | Must | Confirmed |
| FR-08 | Course waypoints and dimensions shall be loaded prior to race start (published ≥4 weeks pre-race). | Must | Confirmed (rules) |
| FR-09 | The swarm shall complete the course autonomously without human control input. | Must | Confirmed |
| FR-10 | The swarm shall navigate vertical gates, horizontal bars, narrowing corridors, tunnels, and elevation changes. | Must | Confirmed (rules) |
| FR-11 | The system shall maintain control through partial occlusions of neighbours and obstacles. VIO broadcast (RF) is occlusion-insensitive; UWB ranging provides occlusion-proof ranging. | Must | Confirmed — radio + UWB recommended |
| FR-12 | The swarm shall remain within course boundary limits at all times. | Must | Confirmed — boundary repulsion in safety filter |

## 3.3 Safety

| ID | Requirement | Priority | Status |
| --- | --- | --- | --- |
| FR-13 | Each drone shall have a hardware emergency stop function on a dedicated channel. | Must | Confirmed (rules) |
| FR-14 | Each drone shall include propeller guards within the 250³ mm envelope. | Must | Confirmed (rules) |
| FR-15 | Minimum safe inter-drone separation shall be maintained at all times (TBC — likely 0.5–1.0 m). The APF safety filter hard floor is currently set at 0.40 m; calibrate to FR-15 once confirmed. | Must | TBC — MATLAB demo holds 0.54 m |
| FR-16 | Each drone shall execute an emergency hover or land if formation cannot be re-established within 2 s. | Must | Confirmed |
| FR-17 | The system shall handle loss of one drone (3-of-4 fallback) without colliding the remaining drones. | Should | TBC |

# 4. Non-Functional Requirements

## 4.1 Timing & Latency

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| NFR-01 | Control loop frequency | ≥30 Hz | Provisional |
| NFR-02 | Policy inference latency | <10 ms per step | TBC — platform benchmark |
| NFR-03 | Sensor-to-action end-to-end latency | <33 ms (1 control cycle) | TBC |
| NFR-04 | VIO pose update rate | ≥30 Hz effective (Mighty: 800 Hz IMU-propagated; 15 Hz visual correction) | Provisional — verify with Mighty on bench |
| NFR-05 | Inter-drone state broadcast rate | ≥20 Hz | TBC |
| NFR-06 | Maximum tolerable inter-drone comms dropout | <100 ms before failsafe trigger | TBC |
| NFR-07 | Formation reform time after any break | ≤2.0 s (mandated by rules) | Confirmed; MATLAB baseline 0.60 s |

## 4.2 Onboard Compute

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| NFR-08 | Onboard RAM | ≥4 GB (VIO + RL inference + perception) | Provisional, platform-dependent |
| NFR-09 | Onboard compute class | Jetson Orin Nano-class or QRB5165-class | TBC — platform selection |
| NFR-10 | Policy inference within latency budget on chosen platform | <NFR-02 | TBC — benchmark |
| NFR-11 | VIO + perception within compute budget co-resident with policy at NFR-01 rate | All components at ≥30 Hz | TBC — benchmark |

## 4.3 Inter-Drone Communications

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| NFR-12 | Inter-drone comms protocol | Radio broadcast of VIO poses primary (Wi-Fi/ESP-NOW); UWB ranging strongly recommended | Radio confirmed; UWB TBC |
| NFR-13 | Bandwidth per drone (state broadcast) | <5 KB/s per drone at 20 Hz | TBC |
| NFR-14 | Reliability in RF-noisy environment | <1% packet loss at operational range | TBC — venue dependent |
| NFR-15 | Failsafe behaviour on comms loss >NFR-06 | Hover or controlled land — must not collide | Must implement |

## 4.4 Mass, Dimensions, Power

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| NFR-16 | Take-off weight | ≤1.5 kg per drone (mandated) | Confirmed (rules) |
| NFR-17 | External dimensions including prop guards | ≤250 × 250 × 250 mm (mandated) | Confirmed (rules) |
| NFR-18 | Battery | LiPo 4S maximum (competition rule) | Confirmed (rules) |
| NFR-19 | Minimum flight duration per charge | TBC — race duration ≤5 min; target ≥8 min with margin | TBC — battery selection |

## 4.5 Sim-to-Real Transfer

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| NFR-20 | Motor constant variation coverage in training | ±10–15% randomisation around measured values | Confirmed |
| NFR-21 | Sensor noise modelling | IMU, camera/VIO noise randomised during training | Confirmed |
| NFR-22 | Mass and drag randomisation | Randomised at episode start during training | Confirmed |
| NFR-23 | Aerodynamic effect of propeller guards | Modelled or measured and incorporated into sim drone model | TBC |
| NFR-24 | Single-drone validation before swarm deployment | Policy validated on one real drone before scaling to 4 | Confirmed |
| NFR-25 | Sim-side velocity controller calibration to chosen autopilot | Sim PID gains and randomisation envelope bracket the chosen autopilot measured velocity step response within ±15% | NEW (v0.3) — bench measurement during Stage 5 prep |

## 4.6 Sensor Suite (NEW)

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| NFR-26 | IMU — 6-axis accelerometer + gyroscope | FC-integrated; supplies angular velocity directly and anchors attitude + velocity estimation | Confirmed — on all shortlisted FCs |
| NFR-27 | Forward camera — global shutter preferred | VIO + gate/obstacle perception + floor-tape anchoring. Stereo preferred (metric scale); mono acceptable with scale calibration. | Confirmed — Mighty Camera (VIO); separate or shared for perception |
| NFR-28 | ESC bidirectional DShot RPM telemetry | Normalised motor RPMs for training obs. Enable in autopilot config on chosen platform. | Confirmed — load-bearing obs |
| NFR-29 | Inter-drone radio link | Broadcast VIO pose at ≥20 Hz (NFR-05). Wi-Fi or ESP-NOW on companion. | Confirmed |
| NFR-30 | Downward optical-flow sensor | Body-frame horizontal velocity (vx, vy) directly. Single biggest improvement to velocity obs accuracy indoors. | Strongly recommended |
| NFR-31 | Downward 1D ToF / lidar rangefinder | Altitude AGL and scale reference for optical flow. | Strongly recommended |
| NFR-32 | UWB ranging modules (one per drone) | Direct metric range to each neighbour (~10 cm noise, occlusion-proof). ~£150–250/unit. | Strongly recommended — decide pre-Stage 5 |
| NFR-33 | Mighty Camera timing risk mitigation | OAK-D Lite (£100/unit) procured as fallback VIO. If Mighty batch 2 not received by 15 July 2026, switch to OAK-D Lite immediately. | NEW — procure OAK-D Lite now |

## 4.7 Classical Controller

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| NFR-34 | Classical controller reform time | Reform after disturbance kick ≤2.0 s | CONFIRMED — **1.25 s** in the C env under full rigid-body dynamics (2026-07-16). The earlier 0.60 s was ideal-dynamics (no actuator lag); plan against ~1.25–1.3 s. See tech doc §8.4 |
| NFR-35 | Classical controller minimum inter-drone separation | ≥d_sep_min (FR-15 TBC, currently 0.40 m floor) | CONFIRMED — **0.488 m** in the C env post-kick (2026-07-16), vs 0.54 m ideal-dynamics. See tech doc §8.4 |
| NFR-36 | Classical controller C port | Logic ported to dronelib.h / velocity_controller.h inside PufferLib env c_step before Stage 3 | **CONFIRMED (2026-07-16)** — `ocean/drone/velocity_controller.h`; reproduces the MuJoCo reference to ~3%. Gated by `stirling/tests/run_velocity_tests.sh`. NB the MATLAB draft was never located; the law was reconstructed as Python (`stirling/controller/`), MuJoCo-validated, then ported |
| NFR-37 | Safety filter wraps controller + residual | APF/CBF filter applied to u_classic + k_res×Δv every cycle; hard separation guarantee regardless of RL residual | **PARTIAL — previously mis-marked as confirmed.** The wrapping is confirmed (filter applies to u_classic + k_res×Δv every cycle, residual added before the filter). The **hard guarantee is NOT met by the APF path** and cannot be: APF fails the 0.40 m floor above ~1.75 m/s head-on closing (`D_ACT×KV/2` — geometry, not tuning; measured 0.042 m at 3 m/s). Tech doc §8.3 only ever claimed a hard guarantee for CBF-QP. Currently satisfied *in practice* because the box formation stays well inside the envelope (0.488 m post-kick), but the requirement as literally worded needs either CBF-QP or a bounded `k_res`. **Decide during the Stage 3b `k_res` sweep.** |

# 5. Perception & Localisation Requirements

## 5.1 Absolute Positioning

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| PR-01 | Each drone shall maintain an absolute position estimate at all times | Mighty Camera VIO primary (15 Hz pose + 800 Hz IMU); OAK-D Lite backup (≥30 Hz). GPS fallback outdoors. | Confirmed — see §9 tech doc |
| PR-02 | Position estimation drift over a single race | <TBC m (depends on race duration & course length). Loop closure required for Mighty to meet tight budgets. | TBC — measure on bench |
| PR-03 | Position update rate | ≥NFR-01 (30 Hz effective via IMU propagation) | Confirmed — 800 Hz IMU propagation bridges 15 Hz visual |
| PR-04 | Floor tape exploitation | System should exploit black/yellow floor tape as a visual anchoring landmark when visible to forward camera | Should — opportunistic |

## 5.2 Inter-Drone Relative Localisation

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| PR-05 | Each drone shall estimate or receive relative positions of all 3 neighbours | Radio broadcast primary (≥20 Hz); UWB ranging cross-check/fallback; visual detection optional sanity-check | Radio confirmed; UWB TBC |
| PR-06 | Relative position accuracy | TBC — formation geometry tolerance. UWB: ~10 cm noise. Radio broadcast: limited by VIO accuracy. | TBC |
| PR-07 | Relative position update rate | ≥NFR-01 | TBC |
| PR-08 | Tolerance to partial occlusion | Must maintain relative position estimates through ≥TBC seconds of single-neighbour visual occlusion. Radio and UWB are occlusion-insensitive. | Confirmed for radio/UWB; TBC for visual-only |
| PR-09 | Dropout handling | Extrapolate from last-known position + velocity for up to NFR-06 (100 ms) before failsafe | Must implement |

## 5.3 Obstacle Perception

| ID | Requirement | Target | Status |
| --- | --- | --- | --- |
| PR-10 | Detection of vertical gates, horizontal bars, narrowing corridors, tunnel entries | Forward camera + depth (stereo or learned monocular depth) | Provisional |
| PR-11 | Free-space estimation in front of drone | Updated at ≥NFR-01 rate | TBC |
| PR-12 | Detection range for course features | TBC — depends on course speed and dimensions | TBC — course publication |

# 6. Items Closed

Onboard vs offboard inference — onboard mandated (rules).
Sensor permissions — onboard only; no external infrastructure permitted (rules).
Hardware permissions — companion compute permitted within 1.5 kg / 250³ mm envelope (rules).
Battery — LiPo 4S maximum (rules).
Run format — A→B, ≤5 min, 2 official runs, best counts (rules).
Race duration / battery endurance — partially answered; target ≥8 min to cover ≤5 min run with margin.
Simulator choice — extended PufferLib 4.0 Ocean drone env (RL pipeline doc v0.3).
Primary action interface — 3-float world-frame velocity setpoints; sim-side velocity wrapper; hardware via PX4 offboard or ArduPilot guided (RL pipeline doc §2.4).
Control approach — residual RL: u_total = clip(u_classic + k_res×Δv, v_max); classical controller mandatory as baseline/fallback/prior (RL pipeline doc §2.5).
Classical controller — C port complete and validated in the PufferLib env (NFR-36 closed 2026-07-16): reform 1.25 s, min sep 0.488 m, worst error 0.76 m, reproducing the MuJoCo reference (1.29 / 0.49 / 0.76) to ~3%. Gated by `stirling/tests/run_velocity_tests.sh`. The original MATLAB draft was never located; the law was reconstructed as Python (`stirling/controller/`), validated in MuJoCo under full rigid-body dynamics, then ported to C. The v0.4 figures (0.60 s / 0.54 m) came from an ideal-dynamics harness and are superseded — the same law costs ~2× the reform time once a real inner loop is in the path.
Formation reframing — box/square is home formation; line/stack/compressed/diamond are transient obstacle-avoidance deviations (June 2026 session).
Sensor suite — IMU, forward camera, ESC DShot RPM, radio broadcast (essential); optical-flow + ToF, UWB (strongly recommended) — NFR-26 through NFR-33 added.
VIO module — Mighty Camera ($60/unit, 10 g, 15 Hz + 800 Hz IMU) primary; OAK-D Lite backup (June 2026 session).
Inter-drone relative localisation architecture — radio broadcast primary; UWB strongly recommended; visual detection optional (June 2026 session).

# 7. Items Still Open

| Question | Why It Matters | How to Resolve | Priority |
| --- | --- | --- | --- |
| Hardware platform selection | Drives all NFRs with platform-dependent targets | Tech doc §3 shortlist shootout | Critical |
| UWB modules — procure and test | Determines PR-08 achievability; protects against occlusion failures in brief | Select modules; bench test inter-drone ranging before Stage 5 | High |
| Mighty Camera batch 2 arrival | If late, VIO integration compresses to weeks before race | Monitor; switch to OAK-D Lite if not received by 15 July 2026 | High |
| VIO drift calibration (Mighty, no LC) | Monocular without loop closure drifts 1–5%. Need to measure actual drift on bench. | Fly known-distance segment; measure odometry error; calibrate scale with UWB if needed | High after hardware selection |
| CBF-QP vs APF safety filter | CBF gives hard separation guarantee; APF is C-portable default | Promote CBF to default and port C QP if collision risk analysis warrants it | Medium |
| NFR-25 calibration — autopilot velocity step response | Closes sim-to-real gap on velocity-tracking interface | Bench measurement on chosen autopilot during Stage 5 prep | High after platform selection |
| Obstacle perception design | PR-10 to PR-12 cannot be quantified without chosen approach | Decide stereo vs learned mono depth on chosen compute | High |
| Inter-drone comms protocol specifics | NFR-12 to NFR-14 depend on chosen radio hardware | Prototype mesh radio test on new platform | High |
| Course speed envelope | Determines NFR-01 sufficiency and motor-level escalation decision | Apply once course published; re-tune if needed | Medium (deferred) |
| FR-15 minimum separation value | Safety filter floor (currently 0.40 m TBC) and reward penalty threshold | Confirm with competition organisers or measure from formation geometry | Medium |
| BOM audit (Crazyflie sunk cost) | Recovers budget for new platform | Resale or repurpose | Medium |

# 8. Validation Criteria

| Test | Pass Criterion | When |
| --- | --- | --- |
| Classical controller smoke test (MATLAB/Octave) | Reform after disturbance kick ≤2.0 s; min sep ≥d_sep_min; formation error reasonable | PASSED — 0.60 s / 0.54 m |
| Classical controller C port unit test | Identical results to MATLAB smoke test in PufferLib env HOVER mode | Before Stage 3 |
| Single-drone hover stability (new platform) | Drone holds position within TBC m for 60 s | After Stage 3 begins |
| VIO drift bench test | Drift within budget over representative course distance | VIO bring-up |
| Autopilot velocity step response (NFR-25) | Measured response within ±15% of sim PID model after calibration | Stage 5 prep |
| UWB ranging accuracy bench test | Range error <15 cm over 0–5 m drone-to-drone at flight speed | Pre-Stage 5 if UWB procured |
| 2-drone formation hold (sim) | Pair holds relative position within TBC m for full simulated lap | Early Stage 4 |
| 4-drone formation hold (sim) | Box mode held over open-course segments | Stage 4 milestone |
| 4-drone mode transitions (sim) | All mode transitions complete within 2.0 s reform window | Stage 4 milestone |
| 4-drone obstacle navigation (sim) | Full course completed including all obstacle types | Stage 4 milestone |
| Single-drone sim-to-real transfer | Policy trained in sim flies stably on real hardware without retraining | Stage 5 hardware validation |
| 4-drone formation hold (real) | Full swarm completes course at competition spec | Final system validation |
| Inference latency benchmark | Policy + VIO + perception within control cycle on target hardware | Pre-deployment |
| Emergency stop | All 4 drones cut motors within <100 ms of trigger | Pre-deployment (mandatory) |

# 9. References

| Document | Relevance |
| --- | --- |
| Formation Racing Drone Swarm — Tech Docs v0.6 | Parent technical document; architecture, sensor suite, VIO platform, classical controller, formation reframing |
| Formation Racing Drone Swarm — RL Pipeline & Environment Plan v0.3 | Residual RL design (§2.5), control composition, sim choice, env extensions, training stages |
| Classical Controller (MATLAB draft) | classical_controller/ folder — smoke-tested formation controller |
| Competition Rules Release (April 2026) | Source of all rule-driven requirements |
| Mighty Camera | mightycamera.com — $60/unit embedded SLAM; batch 2 mid-summer 2026 |
| OAK-D Lite (backup VIO) | luxonis.com — stereo camera + IMU, ≥30 Hz, £100/unit |
| PufferLib | github.com/PufferAI/PufferLib (4.0 branch) |
| PufferLib drone env (origin) | github.com/tensaur/drone — Sam Turner & Finlay Sanders, MIT-licensed |
| Champion-level drone racing (Swift) | Nature 2023, DOI 10.1038/s41586-023-06419-4 |
| Batra et al. — end-to-end MARL swarms | CoRL 2022, PMLR 164:576-586 — PPO formation RL; DeepSets neighbour encoding |
| Distributed MPC formation control | Drones 2025, 9(5):366 — velocity-setpoint formation controller baseline reference |
| PX4 Offboard mode | docs.px4.io/main/en/flight_modes/offboard.html |
| ArduPilot Guided mode | ardupilot.org/copter/docs/ac2_guidedmode.html |

# 10. Requirements Status Summary

| Status | Count | Notes |
| --- | --- | --- |
| Confirmed (firm by rules or design) | ~32 | Rule-driven items + closed design decisions + sensor suite + classical controller results |
| TBC — testing/benchmarking | ~10 | Latency, drift, separation tolerances, detection range, NFR-25 calibration, UWB bench test |
| TBC — platform selection | ~6 | Compute, VIO stack integration, comms protocol, BOM |
| TBC — course publication | ~3 | Endurance, lap count, course speed envelope |
| Total requirements | ~51 | Across functional, non-functional, perception; added NFR-26–37, PR-04, PR-09 |

Priority action: hardware platform selection. This unlocks resolution of the majority of remaining TBC items. Parallel actions: procure OAK-D Lite (VIO backup, available now); preorder Mighty Camera batch 2; evaluate UWB modules.

# Appendix A — Changelog

v0.5 (July 2026): Classical controller C-port session. (1) **NFR-36 CONFIRMED** — C port complete and validated in the PufferLib env, reproducing the MuJoCo reference to ~3%. (2) **NFR-34/NFR-35 re-measured under full rigid-body dynamics** — reform 0.60 → 1.25 s, min sep 0.54 → 0.488 m. Both still pass, but the v0.4 figures were ideal-dynamics and optimistic; the margin against the 2 s rule is ~0.75 s, not ~1.4 s. (3) **NFR-37 downgraded CONFIRMED → PARTIAL** — it was mis-marked. Its "hard separation guarantee regardless of RL residual" is not deliverable by the APF path and never was (tech doc §8.3 only claims it for CBF-QP); APF measurably fails the 0.40 m floor above ~1.75 m/s head-on closing. Satisfied in practice by the box formation staying inside that envelope, but the literal requirement needs CBF-QP or a bounded `k_res` — decide at the Stage 3b `k_res` sweep. (4) §8 Validation updated with the C-port results and the note that the original MATLAB draft was never located. No requirement priorities weakened; one status corrected downward.

v0.4 (June 2026): Classical control session. (1) FR-01 updated — box/square is the competition home formation; all other modes are transient deviations. (2) NFR-26 through NFR-33 added covering the sensor suite (IMU, camera, ESC DShot, radio, optical-flow, ToF, UWB, Mighty Camera timing mitigation). (3) NFR-34 through NFR-37 added for the classical controller (reform time, separation floor, C port, safety filter). (4) PR-04 added (floor tape exploitation). PR-08/PR-09 added (occlusion tolerance, dropout handling). (5) §5.2 Inter-Drone Relative Localisation updated with radio/UWB/visual reliability analysis. (6) §6 Items Closed updated with sensor suite, VIO module, inter-drone architecture, residual RL, classical controller, formation reframing, run format. (7) §7 Open Items updated with UWB procurement, Mighty Camera timing risk, VIO drift calibration, CBF-QP decision. (8) §8 Validation updated with classical controller smoke test result and UWB bench test. LiPo 4S added to §2 context table.

v0.3 (May 2026): Minor revision following RL pipeline doc v0.2. Adds simulator choice and primary action interface to Items Closed. Adds NFR-25. Updates references.

v0.2 (April 2026): Major revision following competition rules release. Onboard inference mandated; 4-drone square; hardware platform reset; VIO replaces GPS+ArUco; obstacle perception requirements added; 2 s reform window codified.

v0.1 (earlier April 2026): Initial requirements draft for 8-drone Crazyflie-based system.
