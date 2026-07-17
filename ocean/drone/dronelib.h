// Originally made by Sam Turner and Finlay Sanders, 2025.
// Included in pufferlib under the original project's MIT license.
// https://github.com/tensaur/drone

#pragma once

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

// Visualisation properties
#define WIDTH 1080
#define HEIGHT 720
#define TRAIL_LENGTH 50

// Crazyflie Physical Constants
// https://github.com/arplaboratory/learning-to-fly
#define BASE_MASS 0.027f         // kg
#define BASE_IXX 3.85e-6f        // kgm²
#define BASE_IYY 3.85e-6f        // kgm²
#define BASE_IZZ 5.9675e-6f      // kgm²
#define BASE_ARM_LEN 0.0396f     // m
#define BASE_K_THRUST 3.16e-10f  // thrust coefficient
#define BASE_K_DRAG 0.005964552f // yaw moment constant
#define BASE_GRAVITY 9.81f       // m/s^2
#define BASE_MAX_RPM 21702.0f    // RPM
// Motor+prop RPM time constant. Upstream shipped 0.15 s, which is sluggish
// even for a Crazyflie 2.1 (real ~0.02-0.05 s) and is not representative of
// the 5"/250-class propulsion this project targets. At 0.15 s the actuator
// lag caps the whole control cascade — the gains needed to meet the 2 s
// formation reform rule (NFR-36) are outright unstable — so the classical
// controller could not meet spec for reasons that were an artifact of this
// constant rather than the control law. See stirling/docs/progress_log.md.
// Lowering it to a realistic 0.05 s is a Stage 2 (platform recalibration)
// change taken early; it alters env dynamics, so Stage 1 baseline metrics
// recorded against 0.15 s do not carry over and were re-established.
#define BASE_K_MOT 0.05f         // s (RPM time constant)

#define BASE_K_ANG_DAMP 0.0f // angular damping coefficient
#define BASE_B_DRAG 0.0f     // linear drag coefficient
#define BASE_MAX_VEL 20.0f   // m/s
#define BASE_MAX_OMEGA 20.0f // rad/s

// Simulation properties
#define GRID_X 30.0f
#define GRID_Y 30.0f
#define GRID_Z 10.0f
#define MARGIN_X (GRID_X - 1)
#define MARGIN_Y (GRID_Y - 1)
#define MARGIN_Z (GRID_Z - 1)
#define RING_RADIUS 2.0f
#define V_TARGET 0.05f

// Core Parameters
#define DT 0.002f // 500 Hz
#define ACTION_SUBSTEPS 5
#define ACTION_DT (DT * (float)ACTION_SUBSTEPS) // 100 Hz

#define DT_RNG 0.0f

// Corner to corner distance
#define MAX_DIST                                                                                   \
    sqrtf((2 * GRID_X) * (2 * GRID_X) + (2 * GRID_Y) * (2 * GRID_Y) + (2 * GRID_Z) * (2 * GRID_Z))

typedef struct Log Log;
struct Log {
    float episode_return;
    float episode_length;
    float rings_passed;
    // Ticks spent inside separation_floor of another drone. Upstream declared
    // and logged this but never incremented it — task (e) wires it to the
    // inter-drone separation check (FR-15).
    float collisions;
    float sep_breach; // episodes ended by a separation breach
    float oob;
    float ring_collision;
    float timeout;
    float score;
    float perf;
    float ema_dist;
    float ema_vel;
    float ema_omega;
    float n;
};

typedef struct {
    float w, x, y, z;
} Quat;

typedef struct {
    float x, y, z;
} Vec3;

typedef struct {
    Vec3 pos;
    Vec3 vel;
    Quat orientation;
    Vec3 normal;
    float radius;
} Target;

typedef struct {
    Vec3 pos[TRAIL_LENGTH];
    int index;
    int count;
} Trail;

typedef struct {
    Vec3 pos;      // global position (x, y, z)
    Vec3 vel;      // linear velocity (u, v, w)
    Quat quat;     // roll/pitch/yaw (phi/theta/psi) as a quaternion
    Vec3 omega;    // angular velocity (p, q, r)
    float rpms[4]; // motor RPMs
} State;

typedef struct {
    Vec3 vel;         // Derivative of position
    Vec3 v_dot;       // Derivative of velocity
    Quat q_dot;       // Derivative of quaternion
    Vec3 w_dot;       // Derivative of angular velocity
    float rpm_dot[4]; // Derivative of motor RPMs
} StateDerivative;

typedef struct {
    float mass;       // kg
    float ixx;        // kgm^2
    float iyy;        // kgm^2
    float izz;        // kgm^2
    float arm_len;    // m
    float k_thrust;   // thrust coefficient (T = k * rpm^2)
    float k_ang_damp; // angular damping coefficient
    float k_drag;     // yaw moment constant (torque-to-thrust ratio style)
    float b_drag;     // linear drag coefficient
    float gravity;    // m/s^2 (positive, world gravity points -z)
    float max_rpm;    // RPM
    float max_vel;    // m/s (observation clamp)
    float max_omega;  // rad/s (observation clamp)
    float k_mot;      // s (motor RPM time constant)
} Params;

typedef struct {
    // core state and parameters
    State state;
    Params params;
    Vec3 prev_pos;

    // current target
    Target* target;

    // classical controller integral state (velocity_controller.h); world
    // frame, [m*s]. Caller-owned so the control law stays a pure function.
    Vec3 integ;

    // Last classical velocity setpoint (velocity_controller.h), world frame
    // [m/s], for the observation builder — the policy sees the baseline its
    // residual is correcting (RL pipeline §2.5). Stays zero on the native
    // motor path, where no classical law runs.
    Vec3 u_classic;

    // Final commanded velocity setpoint (post-residual, post-APF) and its
    // previous value, for the jerk penalty (task e). Both stay zero on the
    // native motor path, so the jerk term is identically zero there.
    Vec3 v_cmd;
    Vec3 prev_v_cmd;

    // Seconds the drone has been outside align_dist of its target, for the
    // NFR-36 2 s rule (task e). Reset to 0 whenever it is aligned.
    float t_unaligned;

    // target buffer
    Target* buffer;
    int buffer_idx;
    int buffer_size;

    // logging utils
    float last_dist_reward;
    float episode_return;
    int episode_length;
    float score;
    float collisions;
    int rings_passed;
    float hover_score;
    float prev_potential;
    float hover_ema;
    float ema_dist;
    float ema_vel;
    float ema_omega;
} Drone;

static inline float clampf(float v, float min, float max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

static inline float rndf(float a, float b, unsigned int* rng) {
    return a + ((float)rand_r(rng) / (float)RAND_MAX) * (b - a);
}

static inline Vec3 add3(Vec3 a, Vec3 b) { return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 sub3(Vec3 a, Vec3 b) { return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 scalmul3(Vec3 a, float b) { return (Vec3){a.x * b, a.y * b, a.z * b}; }

static inline Quat add_quat(Quat a, Quat b) {
    return (Quat){a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline Quat scalmul_quat(Quat a, float b) {
    return (Quat){a.w * b, a.x * b, a.y * b, a.z * b};
}

static inline float dot3(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float norm3(Vec3 a) { return sqrtf(dot3(a, a)); }

static inline void clamp3(Vec3* vec, float min, float max) {
    vec->x = clampf(vec->x, min, max);
    vec->y = clampf(vec->y, min, max);
    vec->z = clampf(vec->z, min, max);
}

static inline void clamp4(float a[4], float min, float max) {
    a[0] = clampf(a[0], min, max);
    a[1] = clampf(a[1], min, max);
    a[2] = clampf(a[2], min, max);
    a[3] = clampf(a[3], min, max);
}

static inline Quat quat_mul(Quat q1, Quat q2) {
    Quat out;
    out.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    out.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    out.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    out.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return out;
}

static inline void quat_normalize(Quat* q) {
    float n = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (n > 0.0f) {
        q->w /= n;
        q->x /= n;
        q->y /= n;
        q->z /= n;
    }
}

static inline Vec3 quat_rotate(Quat q, Vec3 v) {
    Quat qv = (Quat){0.0f, v.x, v.y, v.z};
    Quat tmp = quat_mul(q, qv);
    Quat q_conj = (Quat){q.w, -q.x, -q.y, -q.z};
    Quat res = quat_mul(tmp, q_conj);
    return (Vec3){res.x, res.y, res.z};
}

static inline Quat quat_inverse(Quat q) { return (Quat){q.w, -q.x, -q.y, -q.z}; }

static inline Quat rndquat(unsigned int* rng) {
    float u1 = rndf(0.0f, 1.0f, rng);
    float u2 = rndf(0.0f, 1.0f, rng);
    float u3 = rndf(0.0f, 1.0f, rng);

    float sqrt_1_minus_u1 = sqrtf(1.0f - u1);
    float sqrt_u1 = sqrtf(u1);

    float pi_2_u2 = 2.0f * (float)M_PI * u2;
    float pi_2_u3 = 2.0f * (float)M_PI * u3;

    Quat q;
    q.w = sqrt_1_minus_u1 * sinf(pi_2_u2);
    q.x = sqrt_1_minus_u1 * cosf(pi_2_u2);
    q.y = sqrt_u1 * sinf(pi_2_u3);
    q.z = sqrt_u1 * cosf(pi_2_u3);
    return q;
}

static inline Quat quat_from_axis_angle(Vec3 axis, float angle) {
    float half = angle * 0.5f;
    float s = sinf(half);
    return (Quat){cosf(half), axis.x * s, axis.y * s, axis.z * s};
}

static inline Target rndring(unsigned int* rng, float radius) {
    Target ring = (Target){0};

    ring.pos.x = rndf(-GRID_X + 2 * radius, GRID_X - 2 * radius, rng);
    ring.pos.y = rndf(-GRID_Y + 2 * radius, GRID_Y - 2 * radius, rng);
    ring.pos.z = rndf(-GRID_Z + 2 * radius, GRID_Z - 2 * radius, rng);

    ring.orientation = rndquat(rng);

    Vec3 base_normal = (Vec3){0.0f, 0.0f, 1.0f};
    ring.normal = quat_rotate(ring.orientation, base_normal);

    ring.radius = radius;
    return ring;
}

static inline float rpm_hover(const Params* p) {
    // total thrust = m*g = 4 * k_thrust * rpm^2
    return sqrtf((p->mass * p->gravity) / (4.0f * p->k_thrust));
}

static inline float rpm_min_for_centered_hover(const Params* p) {
    // choose min_rpm so that action=0 -> (min+max)/2 == hover
    float min_rpm = 2.0f * rpm_hover(p) - p->max_rpm;
    if (min_rpm < 0.0f) min_rpm = 0.0f;
    if (min_rpm > p->max_rpm) min_rpm = p->max_rpm;
    return min_rpm;
}

static inline void init_drone(Drone* drone, unsigned int* rng, float dr) {
    drone->params.arm_len = BASE_ARM_LEN * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.mass = BASE_MASS * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.ixx = BASE_IXX * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.iyy = BASE_IYY * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.izz = BASE_IZZ * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.k_thrust = BASE_K_THRUST * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.k_ang_damp = BASE_K_ANG_DAMP * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.k_drag = BASE_K_DRAG * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.b_drag = BASE_B_DRAG * rndf(1.0f - dr, 1.0f + dr, rng);
    drone->params.gravity = BASE_GRAVITY * rndf(0.99f, 1.01f, rng);

    drone->params.max_rpm = BASE_MAX_RPM;
    drone->params.max_vel = BASE_MAX_VEL;
    drone->params.max_omega = BASE_MAX_OMEGA;

    drone->params.k_mot = BASE_K_MOT * rndf(1.0f - dr, 1.0f + dr, rng);

    float hover = rpm_hover(&drone->params);
    for (int i = 0; i < 4; i++)
        drone->state.rpms[i] = hover;

    drone->state.pos = (Vec3){0.0f, 0.0f, 0.0f};
    drone->prev_pos = drone->state.pos;
    drone->state.vel = (Vec3){0.0f, 0.0f, 0.0f};
    drone->state.omega = (Vec3){0.0f, 0.0f, 0.0f};
    drone->state.quat = (Quat){1.0f, 0.0f, 0.0f, 0.0f};
    drone->integ = (Vec3){0.0f, 0.0f, 0.0f};
    drone->u_classic = (Vec3){0.0f, 0.0f, 0.0f};
    drone->v_cmd = (Vec3){0.0f, 0.0f, 0.0f};
    drone->prev_v_cmd = (Vec3){0.0f, 0.0f, 0.0f};
    drone->t_unaligned = 0.0f;
}

static inline void compute_derivatives(State* state, Params* params, float* actions,
                                       StateDerivative* derivatives) {
    float min_rpm = rpm_min_for_centered_hover(params);

    float target_rpms[4];
    for (int i = 0; i < 4; i++) {
        float u = (actions[i] + 1.0f) * 0.5f; // [0,1]
        target_rpms[i] = min_rpm + u * (params->max_rpm - min_rpm);
    }

    float rpm_dot[4];
    for (int i = 0; i < 4; i++) {
        rpm_dot[i] = (1.0f / params->k_mot) * (target_rpms[i] - state->rpms[i]);
    }

    // motor thrusts
    float T[4];
    for (int i = 0; i < 4; i++) {
        float rpm = state->rpms[i];
        if (rpm < 0.0f) rpm = 0.0f;
        T[i] = params->k_thrust * rpm * rpm;
    }

    // body frame net force
    Vec3 F_prop_body = (Vec3){0.0f, 0.0f, T[0] + T[1] + T[2] + T[3]};

    // body frame force -> world frame force
    Vec3 F_prop = quat_rotate(state->quat, F_prop_body);

    // world frame linear drag
    Vec3 F_aero;
    F_aero.x = -params->b_drag * state->vel.x;
    F_aero.y = -params->b_drag * state->vel.y;
    F_aero.z = -params->b_drag * state->vel.z;

    // linear acceleration
    Vec3 v_dot;
    v_dot.x = (F_prop.x + F_aero.x) / params->mass;
    v_dot.y = (F_prop.y + F_aero.y) / params->mass;
    v_dot.z = ((F_prop.z + F_aero.z) / params->mass) - params->gravity;

    // quaternion rates
    Quat omega_q = (Quat){0.0f, state->omega.x, state->omega.y, state->omega.z};
    Quat q_dot = quat_mul(state->quat, omega_q);
    q_dot.w *= 0.5f;
    q_dot.x *= 0.5f;
    q_dot.y *= 0.5f;
    q_dot.z *= 0.5f;

    // body frame torques (plus copter)
    // Vec3 Tau_prop;
    // Tau_prop.x = params->arm_len*(T[1] - T[3]);
    // Tau_prop.y = params->arm_len*(T[2] - T[0]);
    // Tau_prop.z = params->k_drag*(T[0] - T[1] + T[2] - T[3]);

    // body frame torques (cross copter)
    // M1=FR, M2=BR, M3=BL, M4=FL
    // https://www.bitcraze.io/documentation/hardware/crazyflie_2_1_brushless/crazyflie_2_1_brushless-datasheet.pdf
    float arm_factor = params->arm_len / sqrtf(2.0f);
    Vec3 Tau_prop;
    Tau_prop.x = arm_factor * ((T[2] + T[3]) - (T[0] + T[1]));
    Tau_prop.y = arm_factor * ((T[1] + T[2]) - (T[0] + T[3]));
    Tau_prop.z = params->k_drag * (-T[0] + T[1] - T[2] + T[3]);

    // torque from angular damping
    Vec3 Tau_aero;
    Tau_aero.x = -params->k_ang_damp * state->omega.x;
    Tau_aero.y = -params->k_ang_damp * state->omega.y;
    Tau_aero.z = -params->k_ang_damp * state->omega.z;

    // gyroscopic torque
    Vec3 Tau_iner;
    Tau_iner.x = (params->iyy - params->izz) * state->omega.y * state->omega.z;
    Tau_iner.y = (params->izz - params->ixx) * state->omega.z * state->omega.x;
    Tau_iner.z = (params->ixx - params->iyy) * state->omega.x * state->omega.y;

    // angular velocity rates
    Vec3 w_dot;
    w_dot.x = (Tau_prop.x + Tau_aero.x + Tau_iner.x) / params->ixx;
    w_dot.y = (Tau_prop.y + Tau_aero.y + Tau_iner.y) / params->iyy;
    w_dot.z = (Tau_prop.z + Tau_aero.z + Tau_iner.z) / params->izz;

    derivatives->vel = state->vel;
    derivatives->v_dot = v_dot;
    derivatives->q_dot = q_dot;
    derivatives->w_dot = w_dot;
    for (int i = 0; i < 4; i++) {
        derivatives->rpm_dot[i] = rpm_dot[i];
    }
}

static inline void step(State* initial, StateDerivative* deriv, float dt, State* output) {
    output->pos = add3(initial->pos, scalmul3(deriv->vel, dt));
    output->vel = add3(initial->vel, scalmul3(deriv->v_dot, dt));
    output->quat = add_quat(initial->quat, scalmul_quat(deriv->q_dot, dt));
    output->omega = add3(initial->omega, scalmul3(deriv->w_dot, dt));
    for (int i = 0; i < 4; i++) {
        output->rpms[i] = initial->rpms[i] + deriv->rpm_dot[i] * dt;
    }
    quat_normalize(&output->quat);
}

static inline void rk4_step(State* state, Params* params, float* actions, float dt) {
    StateDerivative k1, k2, k3, k4;
    State temp_state;

    compute_derivatives(state, params, actions, &k1);

    step(state, &k1, dt * 0.5f, &temp_state);
    compute_derivatives(&temp_state, params, actions, &k2);

    step(state, &k2, dt * 0.5f, &temp_state);
    compute_derivatives(&temp_state, params, actions, &k3);

    step(state, &k3, dt, &temp_state);
    compute_derivatives(&temp_state, params, actions, &k4);

    float dt_6 = dt / 6.0f;

    state->pos.x += (k1.vel.x + 2.0f * k2.vel.x + 2.0f * k3.vel.x + k4.vel.x) * dt_6;
    state->pos.y += (k1.vel.y + 2.0f * k2.vel.y + 2.0f * k3.vel.y + k4.vel.y) * dt_6;
    state->pos.z += (k1.vel.z + 2.0f * k2.vel.z + 2.0f * k3.vel.z + k4.vel.z) * dt_6;

    state->vel.x += (k1.v_dot.x + 2.0f * k2.v_dot.x + 2.0f * k3.v_dot.x + k4.v_dot.x) * dt_6;
    state->vel.y += (k1.v_dot.y + 2.0f * k2.v_dot.y + 2.0f * k3.v_dot.y + k4.v_dot.y) * dt_6;
    state->vel.z += (k1.v_dot.z + 2.0f * k2.v_dot.z + 2.0f * k3.v_dot.z + k4.v_dot.z) * dt_6;

    state->quat.w += (k1.q_dot.w + 2.0f * k2.q_dot.w + 2.0f * k3.q_dot.w + k4.q_dot.w) * dt_6;
    state->quat.x += (k1.q_dot.x + 2.0f * k2.q_dot.x + 2.0f * k3.q_dot.x + k4.q_dot.x) * dt_6;
    state->quat.y += (k1.q_dot.y + 2.0f * k2.q_dot.y + 2.0f * k3.q_dot.y + k4.q_dot.y) * dt_6;
    state->quat.z += (k1.q_dot.z + 2.0f * k2.q_dot.z + 2.0f * k3.q_dot.z + k4.q_dot.z) * dt_6;

    state->omega.x += (k1.w_dot.x + 2.0f * k2.w_dot.x + 2.0f * k3.w_dot.x + k4.w_dot.x) * dt_6;
    state->omega.y += (k1.w_dot.y + 2.0f * k2.w_dot.y + 2.0f * k3.w_dot.y + k4.w_dot.y) * dt_6;
    state->omega.z += (k1.w_dot.z + 2.0f * k2.w_dot.z + 2.0f * k3.w_dot.z + k4.w_dot.z) * dt_6;

    for (int i = 0; i < 4; i++) {
        state->rpms[i] +=
            (k1.rpm_dot[i] + 2.0f * k2.rpm_dot[i] + 2.0f * k3.rpm_dot[i] + k4.rpm_dot[i]) * dt_6;
    }

    quat_normalize(&state->quat);
}

static inline void move_drone(Drone* drone, float* actions) {
    clamp4(actions, -1.0f, 1.0f);

    for (int s = 0; s < ACTION_SUBSTEPS; s++) {
        rk4_step(&drone->state, &drone->params, actions, DT);

        clamp3(&drone->state.vel, -drone->params.max_vel, drone->params.max_vel);
        clamp3(&drone->state.omega, -drone->params.max_omega, drone->params.max_omega);

        for (int i = 0; i < 4; i++) {
            drone->state.rpms[i] = clampf(drone->state.rpms[i], 0.0f, drone->params.max_rpm);
        }
    }
}

static inline void reset_rings(unsigned int* rng, Target* ring_buffer, int num_rings) {
    ring_buffer[0] = rndring(rng, RING_RADIUS);

    // ensure rings are spaced at least 2*ring_radius apart
    for (int i = 1; i < num_rings; i++) {
        do {
            ring_buffer[i] = rndring(rng, RING_RADIUS);
        } while (norm3(sub3(ring_buffer[i].pos, ring_buffer[i - 1].pos)) < 2.0f * RING_RADIUS);
    }
}

static inline Drone* nearest_drone(Drone* agent, Drone* others, int num_agents) {
    float min_dist = FLT_MAX;
    Drone* nearest = NULL;

    for (int i = 0; i < num_agents; i++) {
        Drone* other = &others[i];
        if (other == agent) continue;

        float dist = norm3(sub3(agent->state.pos, other->state.pos));

        if (dist < min_dist) {
            min_dist = dist;
            nearest = other;
        }
    }

    return nearest;
}

static inline int check_ring(Drone* drone, Target* ring) {
    // previous dot product negative if on the 'entry' side of the ring's plane
    float prev_dot = dot3(sub3(drone->prev_pos, ring->pos), ring->normal);
    float new_dot = dot3(sub3(drone->state.pos, ring->pos), ring->normal);

    bool valid_dir = (prev_dot < 0.0f && new_dot > 0.0f);
    bool invalid_dir = (prev_dot > 0.0f && new_dot < 0.0f);

    // if we have crossed the plane of the ring
    if (valid_dir || invalid_dir) {
        // find intesection with ring's plane
        Vec3 dir = sub3(drone->state.pos, drone->prev_pos);
        float denom = dot3(ring->normal, dir);
        if (fabsf(denom) < 1e-9f) return 0;

        float t = -prev_dot / denom;
        Vec3 intersection = add3(drone->prev_pos, scalmul3(dir, t));
        float dist = norm3(sub3(intersection, ring->pos));

        if (dist < (ring->radius - 0.5f) && valid_dir) {
            return 1;
        } else if (dist < ring->radius + 0.5f) {
            return -1;
        }
    }

    return 0;
}

static inline bool check_collision(Drone* agent, Drone* others, int num_agents) {
    if (num_agents <= 1) return false;

    Drone* nearest = nearest_drone(agent, others, num_agents);
    Vec3 to_nearest = sub3(agent->state.pos, nearest->state.pos);
    float nearest_dist = norm3(to_nearest);

    return nearest_dist < 0.1f;
}

// Velocity is measured *relative to the target* (Stage 3 task e).
//
// These two functions score "is the drone on its target and settled". The
// upstream form used absolute velocity, which is correct only when the target
// is static. A FORMATION drone must cruise with its centroid at ~1 m/s, so
// absolute velocity penalised it for performing the task — measured at 15.2%
// of score, and worse, `reward` contains alpha_hover*potential, so a residual
// would be rewarded for slowing down, i.e. for leaving the formation.
//
// This is exactly identity for every task with a static target: HOVER, ORBIT,
// CUBE and FLAG all set target->vel = 0. IDLE/FOLLOW/CONGO carry the upstream
// per-tick target->vel convention (see tasks.h) and shift slightly; they are
// untrained demo tasks.
//
// omega is deliberately left absolute: the velocity cascade drives yaw rate to
// zero (yaw is damping-only), so drones do not rotate with the formation — the
// slot geometry rotates around them.
float hover_potential(Drone* agent, float hover_dist, float hover_omega, float hover_vel) {
    float dist = norm3(sub3(agent->target->pos, agent->state.pos));
    float vel = norm3(sub3(agent->state.vel, agent->target->vel));
    float omega = norm3(agent->state.omega);

    float d = 1.0f / (1.0f + dist / hover_dist);
    float v = 1.0f / (1.0f + vel / hover_vel);
    float w = 1.0f / (1.0f + omega / hover_omega);

    return d * (0.7f + 0.15f * v + 0.15f * w);
}

// Velocity relative to the target — see hover_potential above.
float check_hover(Drone* agent, float hover_dist, float hover_omega, float hover_vel) {
    float dist = norm3(sub3(agent->target->pos, agent->state.pos));
    float vel = norm3(sub3(agent->state.vel, agent->target->vel));
    float omega = norm3(agent->state.omega);

    float d = dist / (hover_dist * 10.0f);
    float v = vel / (hover_vel * 10.0f);
    float w = omega / (hover_omega * 10.0f);

    float score = 1.0f - 0.7f * d - 0.15f * v - 0.15f * w;
    return score > 0.0f ? score : 0.0f;
}

// --- Observation layout (Stage 3 task d) ------------------------------------
//
// Single source of truth for the obs width; binding.c's OBS_SIZE and the
// Python policy's input dim both derive from this.
//
//   [ 0.. 2] body linear velocity
//   [ 3.. 5] body angular velocity
//   [ 6.. 9] orientation quaternion
//   [10..15] two-scale body-frame offset to target (coarse, fine)
//   [16..18] target normal, body frame
//   [19..27] 3 neighbour relative positions, body frame, by agent index
//   [28..32] formation mode one-hot
//   [33]     time to next formation mode change, tanh-scaled
//   [34..36] u_classic, body frame — the baseline the residual corrects
//   [37..40] motor RPMs  <-- must stay LAST (upstream invariant)
//
// The swarm is 4 drones (tech doc §2), so a drone has exactly 3 neighbours —
// every other agent in its env. At num_agents=1 (the Stage 3a benchmark) there
// are none and the block is zeros, which is the plan's "stub as zeros for
// Stage 3" without a stub: the same code serves Stage 4's num_agents=4 with
// real neighbours. Absent neighbours read as (0,0,0), i.e. co-located; that is
// only ambiguous if presence varies within a run, and it does not — num_agents
// is fixed per env.
#define OBS_BASE 19
#define OBS_N_NEIGHBORS 3
#define OBS_N_FORM_MODES 5 // mirrors FORM_MODE_N in tasks.h (asserted there)
#define DRONE_OBS_SIZE (OBS_BASE + 3 * OBS_N_NEIGHBORS + OBS_N_FORM_MODES + 1 + 3 + 4)

// Body-frame neighbour offsets live on a ~0.4-3 m scale (separation floor to
// formation diameter), so they need their own tanh scale: the target's coarse
// 0.1 is tuned for the 30 m grid and would squash them all to near zero.
#define OBS_NEIGHBOR_SCALE 0.5f
#define OBS_TTM_SCALE 0.5f // time-to-mode-change: tanh(0.5*t), ~0.76 at 2 s
// Canonical velocity-setpoint saturation [m/s]. Lives here, in the header both
// sides include, so there is exactly one definition: velocity_controller.h
// defaults VC_V_MAX to this rather than restating it.
//
// It doubles as the u_classic observation's normaliser. A -D override of
// VC_V_MAX (the seam the test sweeps use) deliberately does *not* move this:
// the observation wants a stable scale across a sweep, and the value is clamped
// anyway, so a swept cascade cannot silently rescale the policy's inputs.
#define OBS_V_MAX 3.0f

// formation_mode < 0 means "not the FORMATION task": the one-hot and the
// time-to-change slot read as zeros. time_to_mode_change is in seconds.
void compute_drone_observations(Drone* agent, Drone* agents, int num_agents, int agent_idx,
                                int formation_mode, float time_to_mode_change,
                                float* observations) {
    int idx = 0;

    // choose the hemisphere with w >= 0
    // to avoid observation sign ambiguity
    Quat q = agent->state.quat;
    //if (q.w < 0.0f) {q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z;}

    Quat q_inv = quat_inverse(q);
    Vec3 linear_vel_body = quat_rotate(q_inv, agent->state.vel);
    Vec3 to_target_world = sub3(agent->target->pos, agent->state.pos);
    Vec3 to_target = quat_rotate(q_inv, to_target_world);

    // we should probably clamp the overall velocity
    float denom = agent->params.max_vel * 1.7320508f; // sqrt(3)
    observations[idx++] = linear_vel_body.x / denom;
    observations[idx++] = linear_vel_body.y / denom;
    observations[idx++] = linear_vel_body.z / denom;

    observations[idx++] = agent->state.omega.x / agent->params.max_omega;
    observations[idx++] = agent->state.omega.y / agent->params.max_omega;
    observations[idx++] = agent->state.omega.z / agent->params.max_omega;

    observations[idx++] = q.w;
    observations[idx++] = q.x;
    observations[idx++] = q.y;
    observations[idx++] = q.z;

    // this is body frame so we have to be careful about scaling
    // because distances are relative to the drone orientation
    observations[idx++] = tanhf(to_target.x * 0.1f);
    observations[idx++] = tanhf(to_target.y * 0.1f);
    observations[idx++] = tanhf(to_target.z * 0.1f);

    observations[idx++] = tanhf(to_target.x * 10.0f);
    observations[idx++] = tanhf(to_target.y * 10.0f);
    observations[idx++] = tanhf(to_target.z * 10.0f);

    Vec3 normal_body = quat_rotate(q_inv, agent->target->normal);
    observations[idx++] = normal_body.x;
    observations[idx++] = normal_body.y;
    observations[idx++] = normal_body.z;

    // Neighbours: every other agent in this env, in agent-index order (a fixed
    // permutation, so a slot always means the same drone). Zero-padded.
    int written = 0;
    for (int j = 0; j < num_agents && written < OBS_N_NEIGHBORS; j++) {
        if (j == agent_idx) continue;
        Vec3 rel = quat_rotate(q_inv, sub3(agents[j].state.pos, agent->state.pos));
        observations[idx++] = tanhf(rel.x * OBS_NEIGHBOR_SCALE);
        observations[idx++] = tanhf(rel.y * OBS_NEIGHBOR_SCALE);
        observations[idx++] = tanhf(rel.z * OBS_NEIGHBOR_SCALE);
        written++;
    }
    for (int j = written; j < OBS_N_NEIGHBORS; j++) {
        observations[idx++] = 0.0f;
        observations[idx++] = 0.0f;
        observations[idx++] = 0.0f;
    }

    // Formation mode one-hot, then time to the next scheduled mode change.
    for (int m = 0; m < OBS_N_FORM_MODES; m++)
        observations[idx++] = (m == formation_mode) ? 1.0f : 0.0f;
    observations[idx++] = (formation_mode < 0) ? 0.0f : tanhf(time_to_mode_change * OBS_TTM_SCALE);

    // The classical baseline this policy's residual is added to (§2.5), body
    // frame and normalised by the setpoint saturation. Zero on the motor path.
    Vec3 u_classic_body = quat_rotate(q_inv, agent->u_classic);
    observations[idx++] = clampf(u_classic_body.x / OBS_V_MAX, -1.0f, 1.0f);
    observations[idx++] = clampf(u_classic_body.y / OBS_V_MAX, -1.0f, 1.0f);
    observations[idx++] = clampf(u_classic_body.z / OBS_V_MAX, -1.0f, 1.0f);

    // rpms should always be last in the obs
    observations[idx++] = agent->state.rpms[0] / agent->params.max_rpm;
    observations[idx++] = agent->state.rpms[1] / agent->params.max_rpm;
    observations[idx++] = agent->state.rpms[2] / agent->params.max_rpm;
    observations[idx++] = agent->state.rpms[3] / agent->params.max_rpm;
}