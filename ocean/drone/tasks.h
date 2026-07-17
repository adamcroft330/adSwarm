// Originally made by Sam Turner and Finlay Sanders, 2025.
// Included in pufferlib under the original project's MIT license.
// https://github.com/tensaur/drone

#pragma once

#include <math.h>
#include <stdlib.h>
#include <strings.h>

#include "dronelib.h"

// FORMATION sits directly after HOVER: it is this project's task, and the
// ones below it are upstream demo targets. HOVER stays at 1 so the shipped
// config default (config/drone.ini `task = 1`) is unchanged; the upstream
// tasks shift down one. Anything selecting a task by name via get_task() is
// unaffected.
typedef enum {
    IDLE,      // 0
    HOVER,     // 1 — config default
    FORMATION, // 2
    ORBIT,     // 3
    FOLLOW,    // 4
    CUBE,      // 5
    CONGO,     // 6
    FLAG,      // 7
    RACE,      // 8
    TASK_N     // Should always be last
} DroneTask;

static char const* TASK_NAMES[TASK_N] = {"idle", "hover", "formation", "orbit", "follow",
                                         "cube", "congo", "flag",      "race"};

DroneTask get_task(char* task_name) {
    for (size_t i = 0; i < TASK_N; i++) {
        if (strcasecmp(TASK_NAMES[i], task_name) == 0) {
            return (DroneTask)i;
        }
    }

    return HOVER;
}

// ============================================================================
// Stirling formation manager (tech doc v0.6 §4.2-§4.3, Stage 3 task c).
//
// Port of stirling/controller/formation_manager.py. Drones are grouped into
// formations of FM_N_SLOTS; each group has a virtual centroid flying a course,
// and each slot's world-frame target is
//
//     p_target = centroid.p + Rz(centroid.yaw) * slot_offset(mode, slot)
//     v_target = centroid.v + Rz(yaw)*d_offset + omega x (Rz(yaw)*offset)
//
// v_target feeds the KFF term of the classical tracking law
// (velocity_controller.h), so a moving or turning centroid is tracked without
// lag. NOTE the units: FORMATION writes target->vel in m/s, which is what the
// velocity stack expects. The upstream IDLE/CONGO tasks instead treat
// target->vel as a per-tick displacement (see move_target below) — those two
// conventions differ by ACTION_DT and must not be mixed. FORMATION never calls
// move_target.
//
// Scope: multi-agent slot assignment and the mode-change scheduler are Stage 4.
// Stage 3 flies a programmatically moving centroid with the mode held at box.
// The blend machinery is ported in full (and exercised by the tests) so the
// Stage 4 scheduler only has to call formation_set_mode().
// ============================================================================

#define FM_N_SLOTS 4

// Order matches formation_manager.py's MODES tuple; do not reorder.
//
// The competition brief splits these into two classes, which the tech doc's
// mode table does not (it calls compressed a "TRANSIENT deviation"):
//
//   BOX FAMILY (box, compressed) — always legal. The brief: "The dimensions of
//   the 'box' can be set by the team, and can change in size as required to
//   best meet the course, but a box structure should try to be maintained."
//   A compressed square is a box at a smaller size, i.e. a permitted resize,
//   not a deviation.
//
//   DEVIATIONS (line, stack, diamond) — not boxes. Legal "only when avoiding
//   obstacles", and "the formation must be re-established within 2 seconds".
//   Each one also costs formation-accuracy points, so they are a last resort
//   for clearing an obstacle, never a routine manoeuvre.
typedef enum {
    FORM_BOX, // home / competition default
    FORM_LINE,
    FORM_STACK,
    FORM_COMPRESSED, // a box resize, NOT a deviation — see above
    FORM_DIAMOND,
    FORM_MODE_N // Should always be last
} FormationMode;

// True for the modes that still satisfy "maintain a box formation": box and
// compressed differ only in scale. Stage 4's obstacle logic and any formation-
// accuracy scoring should gate on this rather than on `mode == FORM_BOX`.
static inline bool formation_mode_is_box(FormationMode mode) {
    return mode == FORM_BOX || mode == FORM_COMPRESSED;
}

static char const* FORMATION_MODE_NAMES[FORM_MODE_N] = {"box", "line", "stack", "compressed",
                                                        "diamond"};

// The observation builder emits a mode one-hot but cannot see this enum
// (dronelib.h is included by this header, not the reverse).
_Static_assert(FORM_MODE_N == OBS_N_FORM_MODES, "OBS_N_FORM_MODES (dronelib.h) must track FORM_MODE_N");

FormationMode get_formation_mode(char* mode_name) {
    for (size_t i = 0; i < FORM_MODE_N; i++) {
        if (strcasecmp(FORMATION_MODE_NAMES[i], mode_name) == 0) {
            return (FormationMode)i;
        }
    }

    return FORM_BOX;
}

// Formation geometry. These match stirling/controller/default_params.py
// (FormationParams). Steady-state spacings are deliberately kept above
// VC_D_ACT (0.70 m) so the APF never fights a held formation.
// -D-overridable, like the cascade gains, so a re-tune needs no header edit.
#ifndef FM_BOX_SIDE
#define FM_BOX_SIDE 1.2f // home box side length [m]
#endif
#ifndef FM_LINE_SPACING
#define FM_LINE_SPACING 1.0f // single-file spacing [m]
#endif
#ifndef FM_STACK_SPACING
#define FM_STACK_SPACING 0.75f // vertical column spacing [m] (> VC_D_ACT)
#endif
#ifndef FM_COMPRESSED_SCALE
#define FM_COMPRESSED_SCALE 0.6f // compressed square = box * scale (sep 0.72 > VC_D_ACT)
#endif
#ifndef FM_DIAMOND_LONG
#define FM_DIAMOND_LONG 1.0f // lead/trail offset [m]
#endif
#ifndef FM_DIAMOND_WIDE
#define FM_DIAMOND_WIDE 0.7f // side-wing offset [m]
#endif
#ifndef FM_BLEND_TIME
#define FM_BLEND_TIME 1.0f // linear mode-transition blend [s]
#endif

// Mode scheduler dwell range [s]. VALIDATION FIXTURE ONLY — see
// formation_next_mode. The MuJoCo reference holds each mode for 3 s; this
// randomises so a fixture cannot be gamed by a learned clock. The floor must
// clear the 1 s blend plus the ~1.3 s reform with margin.
//
// Note these dwells exceed the brief's 2 s reform window, deliberately: the
// fixture holds each mode long enough to measure its settle, which a *legal*
// obstacle deviation would never do. Another reason not to train on it.
#ifndef FM_MODE_DWELL_MIN
#define FM_MODE_DWELL_MIN 3.0f
#endif
#ifndef FM_MODE_DWELL_MAX
#define FM_MODE_DWELL_MAX 6.0f
#endif
// INVARIANT: FM_MODE_DWELL_MIN > FM_BLEND_TIME. The dwell must exceed the
// blend, or the scheduler could retarget a transition that is still running.
// formation_set_mode discards a partial blend (prev_mode := mode, as the Python
// reference does), so interrupting one makes the target position jump from the
// blended offset to the destination's — a step the tracker would chase.
//
// Asserted at runtime by the [mode sched] gate rather than with _Static_assert:
// these are floats, and C11 requires an *integer* constant expression, so a
// float comparison there is invalid. Apple clang accepts it; the Linux
// toolchain that builds for training correctly does not.

// Centroid path planner (Stage 3: a rule-based waypoint cursor).
// Default centroid cruise speed [m/s]. Overridable per-run via
// DroneEnv.formation_speed / --env.formation-speed.
//
// 2.2 is a measured choice, not a guess — the competition scores course time as
// heavily as formation accuracy (6 pts each), and the classical controller has
// no speed policy, so this constant *is* its race pace. The old 1.0 was
// arbitrary and left more than half the achievable speed on the table.
//
// Swept on the classical controller (progress_log.md 2026-07-17). Accuracy and
// separation are NOT what limits speed — with a trackable centroid, worst-case
// error is flat at ~0.38 m and nothing breaches the floor anywhere in range.
// The limit is NFR-36: the reform has to fit in the correction authority left
// under VC_V_MAX (3.0) after cruising, and it falls off a cliff:
//
//   speed   perf   flight_sep  worst_err   reform->box   authority
//    1.0   0.981     1.165       0.350       1.66 s        2.0
//    2.0   0.943     1.166       0.378       1.63 s        1.0
//    2.2   0.932     1.166       0.380       1.61 s        0.8   <- shipped
//    2.4   0.925     1.064       0.382       1.61 s        0.6   <- in spec
//    2.6   0.918     0.977       0.378       2.15 s ✗      0.4   <- fails NFR-36
//    2.8   0.885     0.606       0.785       4.05 s ✗      0.2
//
// 2.4 is the fastest in-spec fixed speed, but 2.2 is shipped: it is two steps
// from the 2.6 cliff rather than one, costs 7% throughput, and leaves 0.8 m/s
// of authority — which the Stage 3b residual also has to fit inside, since it
// is added before the VC_V_MAX clip. It still more than doubles the throughput
// of the old default.
#ifndef FM_CRUISE_SPEED
#define FM_CRUISE_SPEED 2.2f
#endif
#ifndef FM_ARRIVE_GAIN
#define FM_ARRIVE_GAIN 1.0f // decelerate into a waypoint at this rate [1/s]
#endif
// Centroid acceleration limit [m/s^2]. The centroid is a reference trajectory,
// and an unbounded one is not trackable: without this the velocity *vector*
// steps discontinuously at every waypoint capture (the direction flips, and
// the arrival ease-in's low speed jumps straight back to cruise), which the
// tracker can only chase. That showed up as excursions of 1.2 m — the width of
// the whole box — at 2 m/s. The MuJoCo reference ramps its centroid for the
// same reason.
//
// 2.0 m/s^2 is gentle for the platform (tilt_max 35 deg allows g*tan(35) =
// 6.9), and consistent with FM_ARRIVE_GAIN: the ease-in's peak deceleration is
// FM_ARRIVE_GAIN * speed, i.e. 2.0 m/s^2 at a 2 m/s cruise.
#ifndef FM_ACCEL_MAX
#define FM_ACCEL_MAX 2.0f
#endif
#ifndef FM_WP_TOL
#define FM_WP_TOL 0.5f // waypoint capture radius [m]
#endif
#ifndef FM_YAW_KP
#define FM_YAW_KP 2.0f // course-heading P gain [1/s]
#endif
#ifndef FM_YAW_RATE_MAX
#define FM_YAW_RATE_MAX 0.5f // centroid turn-rate limit [rad/s]
#endif
// Episode-start offset from the assigned slot [m]. Bounded by geometry, not
// taste: adjacent box slots are FM_BOX_SIDE apart and each drone offsets by up
// to this much, so two of them close to FM_BOX_SIDE - 2*FM_SPAWN_DIST. That
// must stay above the 0.40 m separation floor (VC_D_FLOOR, which this header
// cannot reference — velocity_controller.h includes it, not the reverse), or
// episodes begin already in breach and the APF spends the first tick digging
// out of a violation it did not cause. At 0.35: 1.2 - 0.7 = 0.5 m of margin.
// Asserted over the random distribution by the [formation spawn] gate.
#ifndef FM_SPAWN_DIST
#define FM_SPAWN_DIST 0.35f
#endif

// Waypoint inset from the world extent. Covers the largest slot offset (line,
// 1.5 m) so no slot target is ever placed outside the flyable volume.
#define FM_INSET 2.0f
#define FM_MARGIN_X (MARGIN_X - FM_INSET)
#define FM_MARGIN_Y (MARGIN_Y - FM_INSET)
#define FM_MARGIN_Z (MARGIN_Z - FM_INSET)

// Sentinel: a blend_t0 far enough in the past that alpha saturates at 1, i.e.
// "no transition in progress" (the reference's -inf).
#define FM_NO_BLEND (-1.0e9f)

// Sentinel for Formation.next_mode_t: no mode change is scheduled. Stage 3 has
// no scheduler (mode is held at box), so this is always the case for now and
// the corresponding observation saturates at "far away". The Stage 4 scheduler
// sets next_mode_t and calls formation_set_mode() when the clock reaches it.
#define FM_NO_SCHEDULE (1.0e9f)
#define FM_T_HORIZON 100.0f // reported time-to-change when nothing is scheduled [s]

typedef struct {
    Vec3 pos;       // virtual formation centre [m]
    Vec3 vel;       // centroid velocity [m/s]
    float yaw;      // course heading [rad]
    float yaw_rate; // [rad/s]
} Centroid;

typedef struct {
    Centroid centroid;
    FormationMode mode;
    FormationMode prev_mode;
    float blend_t0;    // formation-clock time the current transition started [s]
    float t;           // formation clock [s]
    float next_mode_t; // clock time of the next scheduled mode change, or FM_NO_SCHEDULE
    Vec3 waypoint;     // current centroid waypoint [m]
    float speed;       // cruise speed [m/s]
} Formation;

// Formation-frame slot offsets (x forward, y left, z up).
//
// Slot order is chosen so box<->deviation transitions do not cross paths:
// slot 0 front-left, 1 front-right, 2 rear-right, 3 rear-left.
static inline Vec3 formation_slot_offset(FormationMode mode, int slot) {
    float s = 0.5f * FM_BOX_SIDE;
    switch (mode) {
        case FORM_BOX: {
            Vec3 o[FM_N_SLOTS] = {{s, s, 0}, {s, -s, 0}, {-s, -s, 0}, {-s, s, 0}};
            return o[slot];
        }
        case FORM_COMPRESSED: {
            float c = s * FM_COMPRESSED_SCALE;
            Vec3 o[FM_N_SLOTS] = {{c, c, 0}, {c, -c, 0}, {-c, -c, 0}, {-c, c, 0}};
            return o[slot];
        }
        case FORM_LINE: {
            float d = FM_LINE_SPACING;
            Vec3 o[FM_N_SLOTS] = {{1.5f * d, 0, 0}, {0.5f * d, 0, 0}, {-0.5f * d, 0, 0}, {-1.5f * d, 0, 0}};
            return o[slot];
        }
        case FORM_STACK: {
            float d = FM_STACK_SPACING;
            Vec3 o[FM_N_SLOTS] = {{0, 0, 1.5f * d}, {0, 0, 0.5f * d}, {0, 0, -0.5f * d}, {0, 0, -1.5f * d}};
            return o[slot];
        }
        case FORM_DIAMOND: {
            Vec3 o[FM_N_SLOTS] = {{FM_DIAMOND_LONG, 0, 0},
                                  {0, -FM_DIAMOND_WIDE, 0},
                                  {-FM_DIAMOND_LONG, 0, 0},
                                  {0, FM_DIAMOND_WIDE, 0}};
            return o[slot];
        }
        default: return (Vec3){0.0f, 0.0f, 0.0f};
    }
}

// Rz(yaw) * v — heading rotation about world z.
static inline Vec3 rz_rotate(float yaw, Vec3 v) {
    float c = cosf(yaw), s = sinf(yaw);
    return (Vec3){c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

static inline float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// Linear transition blend: 0 at the mode change, 1 once FM_BLEND_TIME elapsed.
static inline float formation_blend_alpha(const Formation* f) {
    return clampf((f->t - f->blend_t0) / FM_BLEND_TIME, 0.0f, 1.0f);
}

// Seconds until the next scheduled mode change, saturating at FM_T_HORIZON
// when none is scheduled (always, until the Stage 4 scheduler lands).
static inline float formation_time_to_mode_change(const Formation* f) {
    if (f->next_mode_t >= FM_NO_SCHEDULE) return FM_T_HORIZON;
    return fmaxf(0.0f, fminf(f->next_mode_t - f->t, FM_T_HORIZON));
}

void formation_set_mode(Formation* f, FormationMode mode) {
    if (mode == f->mode) return;
    f->prev_mode = f->mode;
    f->mode = mode;
    f->blend_t0 = f->t;
}

// Pick the next scheduled mode, for the timer-driven VALIDATION FIXTURE
// (env->formation_modes = 1). This is not the mission profile and must not be
// trained on: the competition brief permits a deviation "only when avoiding
// obstacles", so on an open course a timer-driven deviation is both a rules
// violation and a scored loss ("consistently tight and stable" 6 pts vs
// "frequent deviations" 3). The real trigger is an obstacle, which arrives
// with the Stage 4 primitives; this fixture exists to exercise the blend the
// way demo_formation.py's mode sweep does.
//
// Box is HOME and every deviation returns to it before the next one (tech doc
// §4.2), mirroring the reference's schedule: box -> line -> box -> stack -> ...
//
// That alternation is a SAFETY property, not a stylistic one. A linear blend
// between two formations can pass *inside* the separation floor even when both
// endpoints are legal, because the slot offsets interpolate independently:
// line -> diamond dips to 0.330 m against the 0.40 m floor, from endpoints of
// 1.000 m and 1.221 m. Every box <-> deviation blend stays >= 0.636 m. Routing
// through box is therefore what keeps transitions inside FR-15, and the
// [formation blend safety] gate asserts both halves of that claim.
static inline FormationMode formation_next_mode(const Formation* f, unsigned int* rng) {
    if (f->mode != FORM_BOX) return FORM_BOX;
    // Uniform over the deviations, FORM_LINE..FORM_DIAMOND.
    return (FormationMode)(FORM_LINE + (int)(rand_r(rng) % (FORM_MODE_N - 1)));
}

// World-frame target position and velocity for one slot.
//
// v_target carries the centroid velocity, the blend rate, and the yaw-rate
// term, so the tracking law's feedforward stays exact on a turning course
// mid-transition.
void formation_slot_target(const Formation* f, int slot, Vec3* p_out, Vec3* v_out) {
    float a = formation_blend_alpha(f);
    Vec3 off_a = formation_slot_offset(f->prev_mode, slot);
    Vec3 off_b = formation_slot_offset(f->mode, slot);
    Vec3 off = add3(scalmul3(off_a, 1.0f - a), scalmul3(off_b, a));

    Vec3 r_off = rz_rotate(f->centroid.yaw, off);
    *p_out = add3(f->centroid.pos, r_off);

    Vec3 d_off = {0.0f, 0.0f, 0.0f};
    if (a > 0.0f && a < 1.0f) d_off = scalmul3(sub3(off_b, off_a), 1.0f / FM_BLEND_TIME);

    // omega x r_off, with omega = (0, 0, yaw_rate).
    Vec3 yaw_term = {-f->centroid.yaw_rate * r_off.y, f->centroid.yaw_rate * r_off.x, 0.0f};
    *v_out = add3(add3(f->centroid.vel, rz_rotate(f->centroid.yaw, d_off)), yaw_term);
}

// Episode-start state for one slot: near it, moving with it. HOVER spawns the
// drone anywhere and then places its target nearby; FORMATION cannot do that —
// the slot is wherever the centroid is, so a random grid spawn lands tens of
// metres away and terminates as oob on the first tick. Offset is uniform in a
// ball (same construction as set_target_hover), and the slot velocity is
// matched so the episode starts in formation rather than accelerating into it.
void formation_spawn_state(const Formation* f, int idx, unsigned int* rng, Vec3* pos_out,
                           Vec3* vel_out) {
    Vec3 slot_p, slot_v;
    formation_slot_target(f, idx % FM_N_SLOTS, &slot_p, &slot_v);

    float u = rndf(0.0f, 1.0f, rng);
    float v = rndf(0.0f, 1.0f, rng);
    float z = 2.0f * v - 1.0f;
    float a = 2.0f * (float)M_PI * u;
    float r_xy = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    float rad = FM_SPAWN_DIST * cbrtf(rndf(0.0f, 1.0f, rng));
    Vec3 p = add3(slot_p, (Vec3){rad * r_xy * cosf(a), rad * r_xy * sinf(a), rad * z});

    *pos_out = (Vec3){clampf(p.x, -MARGIN_X, MARGIN_X), clampf(p.y, -MARGIN_Y, MARGIN_Y),
                      clampf(p.z, -MARGIN_Z, MARGIN_Z)};
    *vel_out = slot_v;
}

static void formation_pick_waypoint(Formation* f, unsigned int* rng) {
    f->waypoint = (Vec3){rndf(-FM_MARGIN_X, FM_MARGIN_X, rng), rndf(-FM_MARGIN_Y, FM_MARGIN_Y, rng),
                         rndf(-FM_MARGIN_Z, FM_MARGIN_Z, rng)};
}

void formation_reset(Formation* f, unsigned int* rng, float speed) {
    f->t = 0.0f;
    f->mode = FORM_BOX;
    f->prev_mode = FORM_BOX;
    f->blend_t0 = FM_NO_BLEND;
    // Schedule the first deviation. c_reset overrides this to FM_NO_SCHEDULE
    // when formation_modes = 0 (box-only).
    f->next_mode_t = rndf(FM_MODE_DWELL_MIN, FM_MODE_DWELL_MAX, rng);
    f->speed = (speed > 0.0f) ? speed : FM_CRUISE_SPEED;
    f->centroid.pos = (Vec3){rndf(-FM_MARGIN_X, FM_MARGIN_X, rng), rndf(-FM_MARGIN_Y, FM_MARGIN_Y, rng),
                             rndf(-FM_MARGIN_Z, FM_MARGIN_Z, rng)};
    f->centroid.vel = (Vec3){0.0f, 0.0f, 0.0f};
    f->centroid.yaw = rndf(-(float)M_PI, (float)M_PI, rng);
    f->centroid.yaw_rate = 0.0f;
    formation_pick_waypoint(f, rng);
}

// Advance the centroid one control tick along its waypoint course.
//
// Speed eases down on approach (FM_ARRIVE_GAIN) so the centroid does not
// overshoot and ping-pong, and the heading slews toward the course direction
// under a turn-rate limit. centroid.vel/yaw_rate are set to the rates actually
// applied this tick, which is what keeps the slot feedforward exact.
void formation_step(Formation* f, unsigned int* rng, float dt) {
    f->t += dt;

    // Mode scheduler. next_mode_t == FM_NO_SCHEDULE pins the mode (box-only),
    // which is what env->formation_modes = 0 selects.
    if (f->t >= f->next_mode_t) {
        formation_set_mode(f, formation_next_mode(f, rng));
        f->next_mode_t = f->t + rndf(FM_MODE_DWELL_MIN, FM_MODE_DWELL_MAX, rng);
    }

    Vec3 to_wp = sub3(f->waypoint, f->centroid.pos);
    if (norm3(to_wp) < FM_WP_TOL) {
        formation_pick_waypoint(f, rng);
        to_wp = sub3(f->waypoint, f->centroid.pos);
    }

    float d = norm3(to_wp);
    Vec3 dir = (d > 1e-6f) ? scalmul3(to_wp, 1.0f / d) : (Vec3){0.0f, 0.0f, 0.0f};
    float speed = fminf(f->speed, FM_ARRIVE_GAIN * d);

    // Slew the velocity *vector* under FM_ACCEL_MAX rather than snapping to it.
    // This covers direction changes as well as speed changes — a waypoint
    // capture reverses `dir`, and an unbounded turn is what the tracker cannot
    // follow. centroid.vel remains exactly the rate applied this tick, so the
    // slot feedforward stays exact (asserted by the [formation ff] gate).
    Vec3 v_want = scalmul3(dir, speed);
    Vec3 dv = sub3(v_want, f->centroid.vel);
    float dv_max = FM_ACCEL_MAX * dt;
    float dv_norm = norm3(dv);
    if (dv_norm > dv_max) dv = scalmul3(dv, dv_max / dv_norm);

    f->centroid.vel = add3(f->centroid.vel, dv);
    f->centroid.pos = add3(f->centroid.pos, scalmul3(f->centroid.vel, dt));

    // Heading follows the course made good — the *actual* velocity, not the
    // desired direction, so it cannot lead a turn the centroid has not taken
    // yet. Held when nearly vertical or nearly stopped, where the direction is
    // ill-conditioned.
    Vec3 v = f->centroid.vel;
    float horiz = sqrtf(v.x * v.x + v.y * v.y);
    float rate = 0.0f;
    if (horiz > 1e-2f) {
        float err = wrap_pi(atan2f(v.y, v.x) - f->centroid.yaw);
        rate = clampf(FM_YAW_KP * err, -FM_YAW_RATE_MAX, FM_YAW_RATE_MAX);
        if (fabsf(rate * dt) > fabsf(err)) rate = err / dt; // no overshoot
    }
    f->centroid.yaw_rate = rate;
    f->centroid.yaw = wrap_pi(f->centroid.yaw + rate * dt);
}

void move_target(Drone* agent) {
    agent->target->pos.x += agent->target->vel.x;
    agent->target->pos.y += agent->target->vel.y;
    agent->target->pos.z += agent->target->vel.z;

    if (agent->target->pos.x < -MARGIN_X || agent->target->pos.x > MARGIN_X) {
        agent->target->vel.x = -agent->target->vel.x;
    }
    if (agent->target->pos.y < -MARGIN_Y || agent->target->pos.y > MARGIN_Y) {
        agent->target->vel.y = -agent->target->vel.y;
    }
    if (agent->target->pos.z < -MARGIN_Z || agent->target->pos.z > MARGIN_Z) {
        agent->target->vel.z = -agent->target->vel.z;
    }
}

void set_target_idle(unsigned int* rng, Drone* agent) {
    agent->target->pos =
        (Vec3){rndf(-MARGIN_X, MARGIN_X, rng), rndf(-MARGIN_Y, MARGIN_Y, rng), rndf(-MARGIN_Z, MARGIN_Z, rng)};
    agent->target->vel =
        (Vec3){rndf(-V_TARGET, V_TARGET, rng), rndf(-V_TARGET, V_TARGET, rng), rndf(-V_TARGET, V_TARGET, rng)};
}

void set_target_hover(unsigned int* rng, Drone* agent, float hover_target_dist) {
    // uniform direction on sphere
    float u = rndf(0.0f, 1.0f, rng);
    float v = rndf(0.0f, 1.0f, rng);
    float z = 2.0f * v - 1.0f;
    float a = 2.0f * (float)M_PI * u;
    float r_xy = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    Vec3 dir = (Vec3){r_xy * cosf(a), r_xy * sinf(a), z};

    // uniform radius in ball
    float rad = hover_target_dist * cbrtf(rndf(0.0f, 1.0f, rng));
    Vec3 p = add3(agent->state.pos, scalmul3(dir, rad));

    // clamp to grid bounds
    agent->target->pos = (Vec3){
        clampf(p.x, -MARGIN_X, MARGIN_X),
        clampf(p.y, -MARGIN_Y, MARGIN_Y),
        clampf(p.z, -MARGIN_Z, MARGIN_Z)
    };
    agent->target->vel = (Vec3){0.0f, 0.0f, 0.0f};
}

void set_target_orbit(Drone* agent, int idx, int num_agents) {
    // Fibbonacci sphere algorithm
    float R = 8.0f;
    float phi = M_PI * (sqrt(5.0f) - 1.0f);
    float y = 1.0f - 2 * ((float)idx / (float)num_agents);
    float radius = sqrtf(1.0f - y * y);

    float theta = phi * idx;
    float x = cos(theta) * radius;
    float z = sin(theta) * radius;

    agent->target->pos = (Vec3){R * x, R * z, R * y}; // convert to z up
    agent->target->vel = (Vec3){0.0f, 0.0f, 0.0f};
}

void set_target_follow(unsigned int* rng, Drone* agents, int idx) {
    Drone* agent = &agents[idx];

    if (idx == 0) {
        set_target_idle(rng, agent);
    } else {
        agent->target->pos = agents[0].target->pos;
        agent->target->vel = agents[0].target->vel;
    }
}

void set_target_cube(Drone* agent, int idx) {
    float z = idx / 16;
    idx = idx % 16;
    float x = (float)(idx % 4);
    float y = (float)(idx / 4);
    agent->target->pos = (Vec3){4 * x - 6, 4 * y - 6, 4 * z - 6};
    agent->target->vel = (Vec3){0.0f, 0.0f, 0.0f};
}

void set_target_congo(unsigned int* rng, Drone* agents, int idx) {
    if (idx == 0) {
        set_target_idle(rng, &agents[0]);
        return;
    }

    Drone* follow = &agents[idx - 1];
    Drone* lead = &agents[idx];
    lead->target->pos = follow->target->pos;
    lead->target->vel = follow->target->vel;

    // TODO: Slow hack
    for (int i = 0; i < 40; i++) {
        move_target(lead);
    }
}

void set_target_flag(Drone* agent, int idx) {
    float x = (float)(idx % 8);
    float y = (float)(idx / 8);
    x = 2.0f * x - 7;
    y = 5 - 1.5f * y;
    agent->target->pos = (Vec3){0.0f, x, y};
    agent->target->vel = (Vec3){0.0f, 0.0f, 0.0f};
}

void set_target_race(Drone* agent) { *agent->target = agent->buffer[agent->buffer_idx]; }

// Read one slot's target from the current formation state (the centroid is
// advanced once per tick by formation_step, not here). Agents are assigned to
// slots round-robin; for Stage 3 num_agents may be 1 (single-slot). This is a
// pure read of the shared centroid, so an individual agent resetting mid-
// episode rejoins the live formation without disturbing it.
void set_target_formation(Formation* formation, Drone* agent, int idx) {
    Vec3 p, v;
    formation_slot_target(formation, idx % FM_N_SLOTS, &p, &v);
    agent->target->pos = p;
    agent->target->vel = v;
}

void set_target(unsigned int* rng, DroneTask task, Drone* agents, int idx, int num_agents, float hover_target_dist,
                Formation* formation) {
    Drone* agent = &agents[idx];

    if (task == IDLE) set_target_idle(rng, agent);
    else if (task == HOVER) set_target_hover(rng, agent, hover_target_dist);
    else if (task == ORBIT) set_target_orbit(agent, idx, num_agents);
    else if (task == FOLLOW) set_target_follow(rng, agents, idx);
    else if (task == CUBE) set_target_cube(agent, idx);
    else if (task == CONGO) set_target_congo(rng, agents, idx);
    else if (task == FLAG) set_target_flag(agent, idx);
    else if (task == RACE) set_target_race(agent);
    else if (task == FORMATION) set_target_formation(formation, agent, idx);
}
