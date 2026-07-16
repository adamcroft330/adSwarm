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

// --- Cascade gains, tuned for the sim's Crazyflie constants and its slow
//     k_mot=0.15 s motor lag. Loop bandwidths are deliberately separated
//     ~3x per layer (position << velocity << attitude << motor) so the
//     stack is well-damped; the motor lag caps attitude at ~3 rad/s, which
//     sets the whole ladder. Retune on Stage 2 platform recalibration. -----
// Outer classical law (velocity-setpoint space):
// NOTE: KP is capped low by the sim's sluggish k_mot=0.15 s motors — KP=1.0
// already overshoots. 0.6 is the stable ceiling here; reform *speed* is the
// residual RL's job (Stage 3b), task (a) only needs stable pure-classical
// hover. Stage 2's real platform (faster motors) will lift this.
#define VC_KP 0.6f    // position P gain [1/s] (~0.6 rad/s, tau ~1.7 s)
#define VC_V_MAX 2.0f // velocity-setpoint saturation [m/s]
// Inner loop:
#define VC_KV 2.0f              // velocity P gain [1/s] (~3x above position)
#define VC_TILT_MAX 0.6109f     // 35 deg max commanded tilt
#define VC_KR 12.0f             // attitude P [1/s^2] (wn ~3.5 rad/s)
#define VC_KW 5.0f              // attitude D [1/s] (damping ~0.72)
#define VC_THRUST_FLOOR 0.3f    // min collective, fraction of hover thrust
#define VC_THRUST_CEIL 0.95f    // max collective, fraction of max total

static inline Vec3 cross3(Vec3 a, Vec3 b) {
    return (Vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline Vec3 clip_norm3(Vec3 v, float limit) {
    float n = norm3(v);
    if (n > limit) return scalmul3(v, limit / n);
    return v;
}

// Layer 1: classical velocity setpoint toward the current task target.
// For HOVER (static target) this is the tracking law of tech doc §8.2 with
// v_target = 0 and no integral term (no steady-state disturbance in-sim:
// gravity is fed forward exactly from the drone's own params).
static inline Vec3 classical_velocity_setpoint(const Drone* agent) {
    Vec3 err = sub3(agent->target->pos, agent->state.pos);
    return clip_norm3(scalmul3(err, VC_KP), VC_V_MAX);
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

// One full velocity-mode control tick: classical setpoint + scaled residual,
// saturate, convert to motor actions. dv is the policy's 3-float residual
// (in velocity units after scaling by k_res); k_res = 0 recovers the pure
// classical Stage 3a benchmark exactly.
static inline void velocity_control_step(const Drone* agent, const float* dv, float k_res,
                                         float* motor_actions) {
    Vec3 u = classical_velocity_setpoint(agent);
    if (k_res != 0.0f && dv != NULL) {
        u.x += k_res * dv[0] * VC_V_MAX;
        u.y += k_res * dv[1] * VC_V_MAX;
        u.z += k_res * dv[2] * VC_V_MAX;
        u = clip_norm3(u, VC_V_MAX);
    }
    velocity_to_motor_actions(agent, u, motor_actions);
}
