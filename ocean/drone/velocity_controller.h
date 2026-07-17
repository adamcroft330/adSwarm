// Stirling Stage 3 (task a): velocity-setpoint control stack.
//
// Adds the four-layer stack from the RL pipeline doc §2.3/§2.5 in front of
// the native motor-level interface:
//
//     u_classic = classical law (P toward target)          // every tick
//     u_total   = u_classic + k_res * dv                   // RL residual
//     v_cmd     = clip(u_total, v_max)                     // (APF lands with
//                                                          //  FORMATION task)
//     actions   = velocity_to_motor(v_cmd)                 // lowest layer
//
// The lowest layer converts a world-frame velocity setpoint into the native
// 4-float [-1,1] motor actions consumed by move_drone(), inverting the sim's
// own action->RPM->thrust mapping (compute_derivatives in dronelib.h). It is
// the sim-side stand-in for a PX4-offboard/ArduPilot-guided velocity mode.
//
// This scaffold is gated by DroneEnv.control_mode and changes nothing when
// disabled: control_mode=0 (default) keeps the native motor path
// byte-identical for Stage 1 regression. With control_mode=1 and k_res=0,
// behaviour is pure classical velocity control (the dv=0 passthrough of
// stage3_plan.md task a). The Python blueprint for the full classical
// controller (formation manager, APF safety filter) is stirling/controller/;
// those layers land with the FORMATION task (tasks b/c).

#pragma once

#include "dronelib.h"

// Control modes for DroneEnv.control_mode
#define CONTROL_MODE_MOTOR 0 // native 4-float motor actions (Stage 1 path)
#define CONTROL_MODE_VELOCITY 1 // velocity-setpoint stack (Stage 3 path)

// --- Cascade gains. These match stirling/controller/default_params.py, the
//     configuration validated in MuJoCo (reform 1.29 s, min sep 0.49 m).
//
//     Loop bandwidths are separated per layer so the stack stays damped:
//         position (KP=2)  <  velocity (KV=5)  <  attitude (sqrt(KR)=14.1)
//                                              <  motor (1/k_mot=20 rad/s)
//     Each rung is capped by the one below it, so the platform's motor time
//     constant propagates all the way up to "can it reform in 2 s". The
//     env previously shipped BASE_K_MOT=0.15 s, which forced a detune to
//     KP=0.6/KV=2 and made the 2 s rule unreachable; with a realistic 0.05 s
//     the reference cascade is supportable. See progress_log.md.
//
//     KR and KW must move together: sqrt(KR) sets the attitude natural
//     frequency and KW sets its damping (zeta = KW / (2*sqrt(KR)) = 0.88).
//     Raising KR alone just makes it ring.
//
// Each gain is -D-overridable so stirling/tests/ can sweep the cascade
// without editing this header — the seam a platform re-tune uses.
// Outer classical law (tech doc §8.2), velocity-setpoint space:
//     u = KFF*v_target + KP*e + KI*integ
#ifndef VC_KFF
#define VC_KFF 1.0f   // target-velocity feedforward (prevents lag on a moving target)
#endif
#ifndef VC_KP
#define VC_KP 2.0f    // position P gain [1/s] (tau ~0.5 s; spec §8.2 nominal)
#endif
#ifndef VC_KI
#define VC_KI 0.3f    // light integral [1/s^2]; kept well under KP
#endif
#ifndef VC_I_LIMIT
#define VC_I_LIMIT 0.5f // per-axis integrator clamp [m*s]
#endif
#ifndef VC_V_MAX
#define VC_V_MAX 3.0f // velocity-setpoint saturation [m/s]
#endif
// dronelib.h normalises the u_classic observation by this gain, but cannot
// reference it (this header includes that one, not the reverse). Keep in sync.
_Static_assert(VC_V_MAX == OBS_V_MAX, "OBS_V_MAX (dronelib.h) must track VC_V_MAX");
// Inner loop:
#ifndef VC_KV
#define VC_KV 5.0f              // velocity P gain [1/s] (~2.5x above position)
#endif
#ifndef VC_TILT_MAX
#define VC_TILT_MAX 0.6109f     // 35 deg max commanded tilt
#endif
#ifndef VC_KR
#define VC_KR 200.0f            // attitude P [1/s^2] (wn = sqrt(KR) ~14.1 rad/s)
#endif
#ifndef VC_KW
#define VC_KW 25.0f             // attitude D [1/s] (zeta = KW/(2*sqrt(KR)) ~0.88)
#endif
#define VC_THRUST_FLOOR 0.3f    // min collective, fraction of hover thrust
#define VC_THRUST_CEIL 0.95f    // max collective, fraction of max total

// --- APF safety filter (tech doc §8.3) --------------------------------------
// Repulsion is zero at VC_D_ACT, rising to VC_V_REP_MAX at VC_D_FLOOR and
// growing further below it. VC_D_ACT must stay under the tightest intended
// steady-state formation spacing or the filter fights a held formation.
// These are the spec's numbers (FR-15 separation floor), sized for the real
// 250-class platform — they are generous relative to the sim's Crazyflie
// constants (arm 0.04 m), and are a Stage 2 recalibration target.
#define VC_D_FLOOR 0.40f     // hard separation floor [m]
#define VC_D_ACT 0.70f       // inter-drone repulsion activation distance [m]
#define VC_V_REP_MAX 2.0f    // repulsion speed at the floor [m/s]
#define VC_K_DAMP 0.8f       // closing-rate damping gain
#define VC_BOUND_MARGIN 0.5f // course-boundary repulsion band [m]

static inline Vec3 cross3(Vec3 a, Vec3 b) {
    return (Vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline Vec3 clip_norm3(Vec3 v, float limit) {
    float n = norm3(v);
    if (n > limit) return scalmul3(v, limit / n);
    return v;
}

// Layer 3a — nominal tracking law (tech doc §8.2, port of
// stirling/controller/formation_tracking.py):
//
//     u = KFF*v_target + KP*(p_target - p) + KI*integ
//
// P + feedforward + light-I with conditional anti-windup. Mutates the
// caller-owned integrator on `agent`. For HOVER the target is static, so the
// feedforward term is zero and this reduces to P + light-I.
static inline Vec3 classical_velocity_setpoint(Drone* agent, float dt) {
    Vec3 err = sub3(agent->target->pos, agent->state.pos);

    Vec3 u_raw = add3(add3(scalmul3(agent->target->vel, VC_KFF), scalmul3(err, VC_KP)),
                      scalmul3(agent->integ, VC_KI));
    Vec3 u = clip_norm3(u_raw, VC_V_MAX);

    // Conditional anti-windup: only integrate while the unsaturated command is
    // inside the limit, and clamp the stored integral per axis regardless.
    if (norm3(u_raw) < VC_V_MAX) {
        agent->integ = add3(agent->integ, scalmul3(err, dt));
        clamp3(&agent->integ, -VC_I_LIMIT, VC_I_LIMIT);
    }
    return u;
}

// Repulsion magnitude: 0 at d_act, VC_V_REP_MAX at the floor, and still
// growing below it so violations are pushed out hard.
static inline float apf_ramp(float d) {
    return VC_V_REP_MAX * (VC_D_ACT - d) / (VC_D_ACT - VC_D_FLOOR);
}

// Layer 3c — APF safety filter (tech doc §8.3, port of
// stirling/controller/safety_filter.py). Inter-drone repulsion with
// closing-rate damping, plus course-boundary push-back. No solver dependency.
//
// Obstacle repulsion is intentionally absent: the env has no obstacle
// primitives yet (Stage 4). The Python reference keeps a sphere-list term;
// it slots in here unchanged once those land.
//
// Wraps u_classic + k_res*dv, so the separation guarantee holds even if the
// RL residual misbehaves.
static inline Vec3 safety_filter(Vec3 u, int idx, Drone* agents, int num_agents) {
    Vec3 p_i = agents[idx].state.pos;
    Vec3 v_i = agents[idx].state.vel;
    Vec3 out = u;

    // Inter-drone repulsion.
    for (int j = 0; j < num_agents; j++) {
        if (j == idx) continue;
        Vec3 d_vec = sub3(p_i, agents[j].state.pos);
        float d = norm3(d_vec);
        if (d < 1e-6f || d >= VC_D_ACT) continue;

        Vec3 n = scalmul3(d_vec, 1.0f / d);
        float mag = apf_ramp(d);

        // Closing-rate damping: brake fast approaches before distance alone
        // would. Only the approaching component counts.
        float closing = -dot3(sub3(v_i, agents[j].state.vel), n);
        if (closing > 0.0f) mag += VC_K_DAMP * closing;

        out = add3(out, scalmul3(n, mag));
    }

    // Course-boundary repulsion: per-axis push-back inside the margin band of
    // the world extent (GRID_X/Y/Z from dronelib.h).
    const float lo[3] = {-GRID_X, -GRID_Y, -GRID_Z};
    const float hi[3] = {GRID_X, GRID_Y, GRID_Z};
    float p[3] = {p_i.x, p_i.y, p_i.z};
    float push[3] = {0.0f, 0.0f, 0.0f};
    for (int ax = 0; ax < 3; ax++) {
        float gap_lo = p[ax] - lo[ax];
        if (gap_lo < VC_BOUND_MARGIN)
            push[ax] += VC_V_REP_MAX * (VC_BOUND_MARGIN - gap_lo) / VC_BOUND_MARGIN;
        float gap_hi = hi[ax] - p[ax];
        if (gap_hi < VC_BOUND_MARGIN)
            push[ax] -= VC_V_REP_MAX * (VC_BOUND_MARGIN - gap_hi) / VC_BOUND_MARGIN;
    }
    out = add3(out, (Vec3){push[0], push[1], push[2]});

    return clip_norm3(out, VC_V_MAX);
}

// Lowest layer: world-frame velocity setpoint -> native motor actions.
// Cascade: velocity P + gravity ff -> tilt-limited desired thrust vector ->
// reduced-attitude (tilt-only) PD -> mixer -> per-motor RPM -> [-1,1] action.
static inline void velocity_to_motor_actions(const Drone* agent, Vec3 v_sp, float* actions) {
    const Params* p = &agent->params;
    const State* s = &agent->state;

    // Velocity loop -> desired specific force (world frame).
    Vec3 a_cmd = scalmul3(sub3(v_sp, s->vel), VC_KV);
    Vec3 f_des = scalmul3((Vec3){a_cmd.x, a_cmd.y, a_cmd.z + p->gravity}, p->mass);

    // Tilt limit: cap horizontal thrust against the vertical component.
    float fz_min = VC_THRUST_FLOOR * p->mass * p->gravity;
    if (f_des.z < fz_min) f_des.z = fz_min;
    float h_max = f_des.z * tanf(VC_TILT_MAX);
    float h = sqrtf(f_des.x * f_des.x + f_des.y * f_des.y);
    if (h > h_max) {
        f_des.x *= h_max / h;
        f_des.y *= h_max / h;
    }

    // Reduced-attitude error: rotate body z toward the desired thrust axis.
    // Both expressed in the body frame; the error axis is cross(e3, z_des).
    Quat q_inv = quat_inverse(s->quat);
    float f_norm = norm3(f_des);
    Vec3 z_des_body = quat_rotate(q_inv, scalmul3(f_des, 1.0f / f_norm));
    Vec3 e_axis = cross3((Vec3){0.0f, 0.0f, 1.0f}, z_des_body);

    // Attitude PD, torque normalised by inertia (yaw: damping only).
    Vec3 tau;
    tau.x = p->ixx * (VC_KR * e_axis.x - VC_KW * s->omega.x);
    tau.y = p->iyy * (VC_KR * e_axis.y - VC_KW * s->omega.y);
    tau.z = p->izz * (-VC_KW * s->omega.z);

    // Collective thrust along the actual body z-axis.
    Vec3 z_body = quat_rotate(s->quat, (Vec3){0.0f, 0.0f, 1.0f});
    float max_total = 4.0f * p->k_thrust * p->max_rpm * p->max_rpm;
    float thrust = clampf(dot3(f_des, z_body), fz_min, VC_THRUST_CEIL * max_total);

    // Mixer: invert the cross-copter torque model in compute_derivatives.
    //   tau_x = a*((T2+T3)-(T0+T1)),  tau_y = a*((T1+T2)-(T0+T3)),
    //   tau_z = k_drag*(-T0+T1-T2+T3),  a = arm_len/sqrt(2)
    float af = 4.0f * (p->arm_len / sqrtf(2.0f));
    float kd = 4.0f * p->k_drag;
    float T[4];
    T[0] = 0.25f * thrust - tau.x / af - tau.y / af - tau.z / kd;
    T[1] = 0.25f * thrust - tau.x / af + tau.y / af + tau.z / kd;
    T[2] = 0.25f * thrust + tau.x / af + tau.y / af - tau.z / kd;
    T[3] = 0.25f * thrust + tau.x / af - tau.y / af + tau.z / kd;

    // Per-motor thrust -> RPM target -> native action, inverting
    //   T = k_thrust * rpm^2   and   rpm = min_rpm + (u+1)/2*(max - min).
    float min_rpm = rpm_min_for_centered_hover(p);
    for (int i = 0; i < 4; i++) {
        float t_i = T[i] > 0.0f ? T[i] : 0.0f;
        float rpm = sqrtf(t_i / p->k_thrust);
        rpm = clampf(rpm, min_rpm, p->max_rpm);
        actions[i] = 2.0f * (rpm - min_rpm) / (p->max_rpm - min_rpm) - 1.0f;
    }
}

// One full velocity-mode control tick — the four-layer composition of tech
// doc §8.1 / RL pipeline §2.5 (port of classical_control_step.py):
//
//     u_classic = formation_tracking(target, state)   // every tick
//     u_total   = u_classic + k_res * dv              // RL residual
//     v_cmd     = safety_filter(u_total, ...)         // APF, then saturate
//     actions   = velocity_to_motor(v_cmd)            // lowest layer
//
// dv is the policy's 3-float residual; k_res = 0 recovers pure classical
// control exactly (the Stage 3a benchmark and the runtime fallback). The
// residual is added *before* the safety filter, so the APF's separation
// guarantee holds regardless of what the policy commands.
//
// u_classic is returned via `u_classic_out` (may be NULL) so it can be
// appended to the observation later (RL pipeline §2.5).
static inline void velocity_control_step(int idx, Drone* agents, int num_agents,
                                         const float* dv, float k_res, float dt,
                                         float* motor_actions, Vec3* u_classic_out) {
    Drone* agent = &agents[idx];

    Vec3 u_classic = classical_velocity_setpoint(agent, dt);
    if (u_classic_out != NULL) *u_classic_out = u_classic;
    agent->u_classic = u_classic; // observation builder reads this (§2.5)

    Vec3 u_total = u_classic;
    if (k_res != 0.0f && dv != NULL) {
        u_total.x += k_res * dv[0] * VC_V_MAX;
        u_total.y += k_res * dv[1] * VC_V_MAX;
        u_total.z += k_res * dv[2] * VC_V_MAX;
    }

    Vec3 v_cmd = safety_filter(u_total, idx, agents, num_agents);
    agent->prev_v_cmd = agent->v_cmd; // jerk penalty reads the delta (task e)
    agent->v_cmd = v_cmd;
    velocity_to_motor_actions(agent, v_cmd, motor_actions);
}
