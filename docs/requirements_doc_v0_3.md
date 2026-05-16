**Formation Racing Drone Swarm — Requirements v0.3**

*System Requirements Document — Version 0.3*

Updated May 2026 — RL Pipeline Doc Released

**MINOR REVISION: **RL pipeline doc v0.2 published. Simulator choice closed; action interface (sim and hardware) reaffirmed at velocity setpoints with documented motor-level escalation path. Adds NFR-25 covering sim-side velocity-controller calibration, updates §6 (Items Closed) and §9 (References). No requirement priorities or pass criteria are weakened in this revision.

# 1. Purpose & Scope

This document captures the system requirements for an autonomous formation racing drone swarm. With competition rules released and the RL pipeline document published, several previously-open items are now closed; remaining open items are flagged.

# 2. Project Context

| **Parameter** | **Value** |
| --- | --- |
| Mission | 4 drones in square formation racing an obstacle-rich course, fully AI-controlled |
| Budget | £10,000 total |
| Timeline | ~6 months |
| Team effort | 5 members × 7 hrs/week ≈ 210 hours total |
| Target hardware | TBD — Crazyflie 2.1 invalidated by new rules; reselection in progress |
| Compute location | Onboard only (mandated) |
| Max take-off weight | 1.5 kg per drone |
| Max dimensions | 250 × 250 × 250 mm including propeller guards |
| Mandatory hardware | Propeller guards; emergency stop |
| External infrastructure | None permitted (no motion capture, no base stations) |
| Course features | Vertical gates, horizontal bars, narrowing corridors, tunnels, elevation changes, partial occlusions |
| Course dimensions | Published ≥4 weeks before competition day |
| Simulator | Extended PufferLib 4.0 Ocean drone env (closed by RL pipeline doc v0.2) |

# 3. Functional Requirements

## 3.1 Formation Control

| **ID** | **Requirement** | **Priority** | **Status** |
| --- | --- | --- | --- |
| FR-01 | The system shall maintain 4 drones in a square formation as the default configuration | Must | Confirmed |
| FR-02 | Each drone shall compute its own control actions onboard using local observations and broadcast neighbour state | Must | Confirmed (rules) |
| FR-03 | The formation shall transition between defined formation modes (square, line, stack, compressed, diamond) to navigate obstacles | Must | Confirmed |
| FR-04 | The swarm shall reform to the target formation mode within 2.0 seconds of any break or mode change command | Must | Confirmed (rules) |
| FR-05 | Maximum inter-drone position error from the active mode geometry shall not exceed TBC metres during steady flight | Must | TBC — testing |
| FR-06 | The system shall detect and respond to imminent inter-drone collision within one control cycle | Must | Confirmed |

## 3.2 Race Course Navigation

| **ID** | **Requirement** | **Priority** | **Status** |
| --- | --- | --- | --- |
| FR-07 | The swarm shall navigate a pre-defined race course via waypoints | Must | Confirmed |
| FR-08 | Course waypoints and dimensions shall be loaded prior to race start (course published ≥4 weeks pre-race) | Must | Confirmed (rules) |
| FR-09 | The swarm shall complete the course autonomously without human control input | Must | Confirmed |
| FR-10 | The swarm shall navigate vertical gates, horizontal bars, narrowing corridors, tunnels, and elevation changes | Must | Confirmed (rules) |
| FR-11 | The system shall maintain control through partial occlusions of neighbours and obstacles | Must | Confirmed (rules) |
| FR-12 | The swarm shall remain within course boundary limits at all times | Must | Confirmed |

## 3.3 Safety

| **ID** | **Requirement** | **Priority** | **Status** |
| --- | --- | --- | --- |
| FR-13 | Each drone shall have a hardware emergency stop function on a dedicated channel | Must | Confirmed (rules) |
| FR-14 | Each drone shall include propeller guards within the 250³ mm envelope | Must | Confirmed (rules) |
| FR-15 | Minimum safe inter-drone separation shall be maintained at all times (TBC — likely 0.5–1.0 m) | Must | TBC |
| FR-16 | Each drone shall execute an emergency hover or land if formation cannot be re-established within 2 s | Must | Confirmed |
| FR-17 | The system shall handle loss of one drone (3-of-4 fallback) without colliding the remaining drones | Should | TBC |

# 4. Non-Functional Requirements

## 4.1 Timing & Latency

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| NFR-01 | Control loop frequency | ≥ 30 Hz (raised from 10 Hz to support obstacle navigation) | Provisional |
| NFR-02 | Policy inference latency | < 10 ms per step | TBC — needs platform benchmark |
| NFR-03 | Sensor-to-action end-to-end latency | < 33 ms (1 control cycle) | TBC |
| NFR-04 | VIO update rate | ≥ 30 Hz | TBC — VIO stack benchmark |
| NFR-05 | Inter-drone state broadcast rate | ≥ 20 Hz | TBC |
| NFR-06 | Maximum tolerable inter-drone comms dropout | < 100 ms before failsafe trigger | TBC |
| NFR-07 | Formation reform time after any break | ≤ 2.0 s (mandated by rules) | Confirmed (rules) |

## 4.2 Onboard Compute

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| NFR-08 | Onboard RAM | ≥ 4 GB (to host VIO + RL inference + perception) | Provisional, platform-dependent |
| NFR-09 | Onboard compute class | Jetson Orin Nano-class or QRB5165-class | TBC — platform selection |
| NFR-10 | Policy inference within latency budget on chosen platform | < NFR-02 | TBC — benchmark |
| NFR-11 | VIO + perception within compute budget | VIO + obstacle perception co-resident with policy at NFR-01 rate | TBC — benchmark |

## 4.3 Inter-Drone Communications

With base stations forbidden, comms is now exclusively between drones. Approach is undecided (mesh radio vs visual neighbour detection vs hybrid).

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| NFR-12 | Inter-drone comms protocol | TBD — ESPNOW / Wi-Fi mesh / LoRa / visual-only | TBC — design |
| NFR-13 | Bandwidth per drone (state broadcast) | TBC — est. < 5 KB/s per drone at 20 Hz | TBC |
| NFR-14 | Reliability in RF-noisy environment | < 1% packet loss at operational range | TBC — venue dependent |
| NFR-15 | Failsafe behaviour on comms loss > NFR-06 | Hover or controlled land — must not collide | Must implement |

## 4.4 Mass, Dimensions, Power

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| NFR-16 | Take-off weight | ≤ 1.5 kg per drone (mandated) | Confirmed (rules) |
| NFR-17 | External dimensions including prop guards | ≤ 250 × 250 × 250 mm (mandated) | Confirmed (rules) |
| NFR-18 | Minimum flight duration per charge | TBC — race duration unknown until course published | TBC |
| NFR-19 | Battery swap time between heats | TBC | TBC |

## 4.5 Sim-to-Real Transfer

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| NFR-20 | Motor constant variation coverage in training | ± 10–15% randomisation around measured values | Confirmed |
| NFR-21 | Sensor noise modelling | IMU and camera/VIO noise randomised during training | Confirmed |
| NFR-22 | Mass and drag randomisation | Randomised at episode start during training | Confirmed |
| NFR-23 | Aerodynamic effect of propeller guards | Modelled or measured and incorporated into sim drone model | TBC |
| NFR-24 | Single-drone validation before swarm deployment | Policy validated on one real drone before scaling to 4 | Confirmed |
| NFR-25 | Sim-side velocity controller calibration to chosen autopilot | Sim PID gains and randomisation envelope bracket the chosen autopilot's measured velocity step response within ±15% | NEW — required by RL pipeline doc §2.3 / §5.2; bench measurement during Stage 5 prep |

# 5. Perception & Localisation Requirements

## 5.1 Absolute Positioning

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| PR-01 | Each drone shall maintain an absolute position estimate at all times | Visual-Inertial Odometry primary; GPS fallback where signal available | Confirmed |
| PR-02 | Position estimation drift over a single race | < TBC m (depends on race duration & course length) | TBC |
| PR-03 | Position update rate | ≥ NFR-01 (30 Hz) | Confirmed |

## 5.2 Relative Localisation (Inter-Drone)

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| PR-04 | Each drone shall estimate or receive relative positions of all 3 neighbours | Mesh radio broadcast with optional visual cross-check | TBC — design |
| PR-05 | Relative position accuracy | TBC — formation geometry tolerance | TBC |
| PR-06 | Relative position update rate | ≥ NFR-01 | TBC |
| PR-07 | Tolerance to partial occlusion | Must continue formation control through ≥ TBC seconds of single-neighbour occlusion | TBC |

## 5.3 Obstacle Perception

| **ID** | **Requirement** | **Target** | **Status** |
| --- | --- | --- | --- |
| PR-08 | Detection of vertical gates, horizontal bars, narrowing corridors, tunnel entries | Forward-facing camera + depth (stereo or learned monocular) | Provisional |
| PR-09 | Free-space estimation in front of drone | Updated at ≥ NFR-01 rate | TBC |
| PR-10 | Detection range for course features | TBC — depends on course speed and dimensions | TBC |

# 6. Items Closed

The following questions, open in earlier versions, are now closed:

- Onboard vs offboard inference — onboard mandated (rules).

- Sensor permissions — onboard only; no external infrastructure (motion capture, base stations) permitted (rules).

- Hardware permissions on each drone — companion compute permitted within 1.5 kg / 250³ mm envelope (rules).

- Race duration / battery endurance — partially answered: course dimensions arrive ≥4 weeks pre-race.

- Simulator choice — extended PufferLib 4.0 Ocean drone environment (closed by RL pipeline doc v0.2).

- Primary action interface (sim and hardware) — 3-float world-frame velocity setpoints with deterministic yaw; sim-side velocity wrapper inside the extended env, hardware-side via PX4 offboard or ArduPilot guided mode (closed by RL pipeline doc v0.2 §2.4).

- Motor-level escalation policy — held open as a contingent upgrade with documented triggers, not a parallel active path (RL pipeline doc v0.2 §2.4).

# 7. Items Still Open

| **Question** | **Why It Matters** | **How to Resolve** | **Priority** |
| --- | --- | --- | --- |
| Hardware platform selection | Drives every NFR with a platform-dependent target | Tech doc v0.5 §3 — shortlist shootout | Critical |
| VIO stack selection | Determines NFR-04, PR-01, perception compute load | Bench test 2–3 candidates on shortlisted platform | High |
| Inter-drone comms protocol | Determines NFR-12 to NFR-15 and reform-time achievability | Prototype mesh radio test on new platform | High |
| Obstacle perception design | PR-08 to PR-10 cannot be quantified without a chosen approach | Decide stereo vs learned mono depth on chosen compute | High |
| Course speed envelope | Determines NFR-01 sufficiency and informs motor-level escalation decision | Apply once course is published; re-tune if needed | Medium (deferred) |
| Number of laps / race duration | Determines NFR-18 endurance | Course publication | Medium (deferred) |
| BOM audit (Crazyflie sunk cost) | Recovers budget for new platform | Resale or repurpose | Medium |
| NFR-25 calibration — autopilot velocity step response measurement | Closes the sim-to-real gap on the velocity-tracking interface | Bench measurement on chosen autopilot during Stage 5 prep (RL pipeline doc §7) | High (after platform selection) |

# 8. Validation Criteria

| **Test** | **Pass Criterion** | **When** |
| --- | --- | --- |
| Single-drone hover stability (new platform) | Drone holds position within TBC m for 60 s | After Stage 3 begins |
| Single-drone obstacle traversal | Drone clears each obstacle type at racing speed | Hardware milestone |
| VIO drift bench test | Drift within budget over a representative course distance | VIO bring-up |
| Autopilot velocity step response measurement (NFR-25) | Measured response within ±15% of sim PID model after calibration | Stage 5 prep |
| 2-drone formation hold (sim) | Pair holds relative position within TBC m for full simulated lap | Early training |
| 4-drone formation hold (sim) | Square mode held over open-course segments | Training milestone |
| 4-drone mode transitions (sim) | All mode transitions complete within 2.0 s reform window | Training milestone |
| 4-drone obstacle navigation (sim) | Full course completed including all obstacle types | Training milestone |
| Single-drone sim-to-real transfer | Policy trained in sim flies stably on real hardware without retraining | Hardware validation |
| 4-drone formation hold (real) | Full swarm completes course at competition spec | Final system validation |
| Inference latency benchmark | Policy + VIO + perception within control cycle on target hardware | Pre-deployment |
| Emergency stop | All 4 drones cut motors within < 100 ms of trigger | Pre-deployment (mandatory) |

# 9. References

| **Document** | **Relevance** |
| --- | --- |
| Formation Racing Drone Swarm — Project Technical Documentation v0.5 | Parent technical document; architecture decisions and rationale |
| Formation Racing Drone Swarm — RL Pipeline & Environment Plan v0.2 | Sim choice, env extension, action-space rationale, training stages, sim-to-real plan |
| Competition Rules Release (April 2026) | Source of all rule-driven requirements |
| PufferLib | github.com/PufferAI/PufferLib (4.0 branch) |
| PufferLib drone env (origin) | github.com/tensaur/drone — Sam Turner & Finlay Sanders, MIT-licensed |
| gym-pybullet-drones | github.com/utiasDSL/gym-pybullet-drones (contingency) |
| AttentionSwarm | arXiv:2503.07376 (2025) — shared-policy RL on real drone swarms |
| Champion-level drone racing | Nature 2023, DOI 10.1038/s41586-023-06419-4 |
| Domain randomisation theory | arXiv:2110.03239 |
| PX4 Offboard mode | docs.px4.io/main/en/flight_modes/offboard.html |
| ArduPilot Guided mode | ardupilot.org/copter/docs/ac2_guidedmode.html |

# 10. Requirements Status Summary

| **Status** | **Count** | **Notes** |
| --- | --- | --- |
| Confirmed (firm by rules or design) | ~24 | Rule-driven items + carry-over confirmed items + items closed by RL pipeline doc |
| TBC — testing/benchmarking | ~10 | Latency, drift, separation tolerances, obstacle detection range, NFR-25 calibration |
| TBC — platform selection | ~6 | Compute, VIO stack, comms protocol, BOM |
| TBC — course publication | ~3 | Endurance, lap count, course speed envelope |
| Total requirements | ~42 | Across functional, non-functional, perception |

*Priority action: **hardware platform selection. This unlocks resolution of the majority of remaining TBC items, including NFR-25 calibration, and is the critical path to all subsequent development.*

# Appendix A — Changelog

***v0.3 (May 2026): ****Minor revision following RL pipeline doc v0.2 release. Adds simulator choice and primary action interface to §6 (Items Closed). Adds NFR-25 (sim-side velocity controller calibration to chosen autopilot). Updates §7 with the calibration question. Adds a validation row for autopilot velocity step response measurement. References list updated.*

***v0.2 (April 2026): ****Major revision following competition rules release. Onboard inference mandated; 4-drone square (was 8); hardware platform reset; VIO replaces GPS+ArUco for primary localisation; obstacle perception requirements added; 2 s reform window codified; mandatory propeller guards and emergency stop; mass and dimension caps locked.*

***v0.1 (earlier April 2026): ****Initial requirements draft for 8-drone Crazyflie-based system with offboard inference as the open architectural question.*
