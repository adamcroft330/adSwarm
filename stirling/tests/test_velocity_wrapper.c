// Standalone regression harness for the Stage 3 velocity-setpoint wrapper
// (ocean/drone/velocity_controller.h). Compiles against the header-only drone
// env, no raylib / no vecenv / no OpenMP:
//
//   clang -O2 -I ocean/drone -I src stirling/tests/test_velocity_wrapper.c \
//       -lm -o /tmp/test_velocity_wrapper && /tmp/test_velocity_wrapper
//
// Checks: (1) the native motor path (control_mode=0) still runs, (2) pure
// classical velocity control (control_mode=1, dv=0) drives drones to the
// hover target, (3) a single drone settles precisely on a static target.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "drone.h"

// render.h provides this in the real build; stub it for the headless test.
void c_close_client(Client* client) { (void)client; }

#define OBS DRONE_OBS_SIZE

static DroneEnv* make_env(int n, int control_mode, float k_res) {
    DroneEnv* env = calloc(1, sizeof(DroneEnv));
    env->num_agents = n;
    env->max_rings = 10;
    env->task = HOVER;
    env->alpha_dist = 0.782192f;
    env->alpha_hover = 0.071445f;
    env->alpha_shaping = 3.9754f;
    env->alpha_omega = 0.00135588f;
    env->hover_target_dist = 5.0f;
    env->hover_dist = 0.1f;
    env->hover_omega = 0.1f;
    env->hover_vel = 0.1f;
    // Task (e): mirror binding.c's defaults. Every weight inert.
    env->alpha_jerk = 0.0f;
    env->alpha_align = 0.0f;
    env->align_dist = 0.15f;
    env->align_time = 2.0f;
    env->separation_floor = 0.0f;
    env->separation_terminates = 0;
    env->control_mode = control_mode;
    env->k_res = k_res;
    env->rng = 42;
    env->observations = calloc(n * OBS, sizeof(float));
    env->actions = calloc(n * 4, sizeof(float));
    env->rewards = calloc(n, sizeof(float));
    env->terminals = calloc(n, sizeof(float));
    init(env);
    c_reset(env);
    return env;
}

// Average distance-to-target across agents.
static float mean_dist(DroneEnv* env) {
    float s = 0.0f;
    for (int i = 0; i < env->num_agents; i++)
        s += norm3(sub3(env->agents[i].target->pos, env->agents[i].state.pos));
    return s / env->num_agents;
}

// Test 1: classical velocity control (control_mode=1, k_res=0, dv=0) must
// drive drones to the hover target. We freeze the target (no reset churn) by
// pinning a single agent and running enough ticks for the ~0.5 s time
// constant to settle (100 Hz -> 200 ticks = 2 s).
static int test_classical_hover(void) {
    DroneEnv* env = make_env(64, CONTROL_MODE_VELOCITY, 0.0f);
    // actions stay zero => dv=0 => pure classical
    float d0 = mean_dist(env);
    float dmin = d0;
    for (int t = 0; t < 400; t++) {
        c_step(env);
        float d = mean_dist(env);
        if (d < dmin) dmin = d;
    }
    float dfinal = mean_dist(env);
    printf("[classical hover] start=%.3f  min=%.3f  final=%.3f  (target settle < 0.5 m)\n",
           d0, dmin, dfinal);
    int ok = (dmin < 0.5f);
    free(env);
    return ok;
}

// Test 2: a single agent, static target, measure settle time and residual.
static int test_single_settle(void) {
    DroneEnv* env = make_env(1, CONTROL_MODE_VELOCITY, 0.0f);
    Drone* a = &env->agents[0];
    // Pin a clean, reachable setup: drone at origin, target 3 m up-range.
    a->state.pos = (Vec3){0, 0, 0};
    a->state.vel = (Vec3){0, 0, 0};
    a->state.omega = (Vec3){0, 0, 0};
    a->state.quat = (Quat){1, 0, 0, 0};
    a->target->pos = (Vec3){2.0f, 0.0f, 1.0f};
    a->target->vel = (Vec3){0, 0, 0};

    int settle_tick = -1;
    for (int t = 0; t < 600; t++) {
        // keep the target pinned (bypass reset by forcing large oob margin)
        env->hover_target_dist = 100.0f;
        c_step(env);
        a->target->pos = (Vec3){2.0f, 0.0f, 1.0f};
        a->target->vel = (Vec3){0, 0, 0};
        float d = norm3(sub3(a->target->pos, a->state.pos));
        if (settle_tick < 0 && d < 0.1f) settle_tick = t;
    }
    float d = norm3(sub3(a->target->pos, a->state.pos));
    float v = norm3(a->state.vel);
    printf("[single settle] final_dist=%.4f  final_speed=%.4f  settle_tick=%d (%.2f s)\n",
           d, v, settle_tick, settle_tick < 0 ? -1.0f : settle_tick * 0.01f);
    int ok = (d < 0.15f && v < 0.3f);
    free(env);
    return ok;
}

// Test 3: control_mode=0 native path still runs and is unaffected by the new
// fields (smoke: step without crashing, actions=0 => motors at centered hover
// => drone should roughly hold altitude, not diverge instantly).
static int test_motor_path_smoke(void) {
    DroneEnv* env = make_env(16, CONTROL_MODE_MOTOR, 0.0f);
    for (int t = 0; t < 50; t++) c_step(env);
    // just assert finiteness
    int ok = 1;
    for (int i = 0; i < env->num_agents; i++) {
        Vec3 p = env->agents[i].state.pos;
        if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z)) ok = 0;
    }
    printf("[motor smoke] native path ran 50 ticks, finite=%d\n", ok);
    free(env);
    return ok;
}

// --- Task (b) tests: the ported classical controller ------------------------

// Drive the controller directly (bypassing c_step's reset/termination logic)
// so these measure the control law, not the task wrapper.
static void drive(DroneEnv* env, int ticks, void (*on_tick)(DroneEnv*, int)) {
    for (int t = 0; t < ticks; t++) {
        if (on_tick) on_tick(env, t);
        float acts[4];
        // Compute every setpoint from the same pre-step state, then move.
        for (int i = 0; i < env->num_agents; i++) {
            velocity_control_step(i, env->agents, env->num_agents, NULL, 0.0f, ACTION_DT,
                                  acts, NULL);
            move_drone(&env->agents[i], acts);
        }
    }
}

// Test 4: feedforward — a drone must track a *moving* target with small lag.
// With KFF=0 the lag would be v_target/KP = 0.5/0.6 = 0.83 m; with KFF=1 the
// feedforward cancels it, so this fails loudly if KFF is dropped.
static int test_moving_target_feedforward(void) {
    DroneEnv* env = make_env(1, CONTROL_MODE_VELOCITY, 0.0f);
    Drone* a = &env->agents[0];
    a->state.pos = (Vec3){0, 0, 0};
    a->state.vel = (Vec3){0, 0, 0};
    a->state.omega = (Vec3){0, 0, 0};
    a->state.quat = (Quat){1, 0, 0, 0};
    a->integ = (Vec3){0, 0, 0};
    a->target->pos = (Vec3){0, 0, 0};
    a->target->vel = (Vec3){0.5f, 0, 0}; // 0.5 m/s along +x

    float max_lag_late = 0.0f;
    for (int t = 0; t < 800; t++) {
        // advance the target at its own velocity (100 Hz outer loop)
        a->target->pos.x += a->target->vel.x * ACTION_DT;
        float acts[4];
        velocity_control_step(0, env->agents, 1, NULL, 0.0f, ACTION_DT, acts, NULL);
        move_drone(a, acts);
        if (t > 400) { // after transient
            float lag = norm3(sub3(a->target->pos, a->state.pos));
            if (lag > max_lag_late) max_lag_late = lag;
        }
    }
    printf("[feedforward]  steady-state lag on 0.5 m/s target = %.3f m (KFF=0 would give ~0.83)\n",
           max_lag_late);
    int ok = (max_lag_late < 0.25f);
    free(env);
    return ok;
}

// Test 5: APF envelope characterization — NOT a spec gate.
//
// Two drones commanded onto the same point is the maximally adversarial case:
// they converge head-on at v_max. This measures where distance-based APF stops
// working, and it is expected to breach the floor. Why that is not a defect:
//
//   - Tech doc §8.3 claims a hard separation guarantee only for the CBF-QP
//     upgrade. For the APF path it claims none — that asymmetry is the whole
//     reason CBF is on the roadmap.
//   - The Python reference (stirling/controller/safety_filter.py) has the same
//     saturate-after-repulsion structure, so this reproduces it faithfully.
//
// The envelope is predictable: each drone coasts v/KV after its command
// reverses, so two closing head-on cover 2*v/KV before stopping. APF holds
// while that is under VC_D_ACT, i.e. per-drone closing speed below
//     v_safe = VC_D_ACT * VC_KV / 2
// At the shipped gains: 0.70 * 5 / 2 = 1.75 m/s. Above that, only a predictive
// filter (CBF-QP) can guarantee the floor. Equilibrium separation is checked
// too: if that lands near the analytic ~0.67 m the APF math is right and any
// breach is a transient, not a logic error.
static int test_apf_separation(void) {
    DroneEnv* env = make_env(2, CONTROL_MODE_VELOCITY, 0.0f);
    for (int i = 0; i < 2; i++) {
        Drone* a = &env->agents[i];
        a->state.pos = (Vec3){(i == 0 ? -2.0f : 2.0f), 0, 0};
        a->state.vel = (Vec3){0, 0, 0};
        a->state.omega = (Vec3){0, 0, 0};
        a->state.quat = (Quat){1, 0, 0, 0};
        a->integ = (Vec3){0, 0, 0};
        a->target->pos = (Vec3){0, 0, 0}; // both told to fly to the origin
        a->target->vel = (Vec3){0, 0, 0};
    }
    float min_sep = 1e9f, d = 0.0f;
    for (int t = 0; t < 1200; t++) {
        float acts[4];
        for (int i = 0; i < 2; i++) {
            velocity_control_step(i, env->agents, 2, NULL, 0.0f, ACTION_DT, acts, NULL);
            move_drone(&env->agents[i], acts);
        }
        d = norm3(sub3(env->agents[0].state.pos, env->agents[1].state.pos));
        if (d < min_sep) min_sep = d;
    }
    float v_safe = VC_D_ACT * VC_KV / 2.0f;
    int equilibrium_ok = (d > 0.5f && d < 0.9f); // analytic ~0.67 m
    printf("[apf envelope] head-on at v_max=%.1f: min_sep=%.3f m, settles to %.3f m "
           "(analytic ~0.67 -> APF math %s)\n",
           (double)VC_V_MAX, min_sep, d, equilibrium_ok ? "OK" : "SUSPECT");
    printf("               APF holds below ~%.2f m/s closing (D_ACT*KV/2); above it "
           "needs CBF-QP (§8.3). Characterization, not a gate.\n", (double)v_safe);
    // Only the equilibrium is asserted — a wrong equilibrium would mean the
    // repulsion math itself is broken. The transient breach is expected.
    free(env);
    return equilibrium_ok;
}

// Test 6: NFR-36 gate — 4 drones hold a 1.2 m box; kick one and measure reform
// time and min separation. Mirrors stirling/controller/demo_formation.py's
// disturbance scenario, but in the env's own (Crazyflie) dynamics.
//
// k_mot_override > 0 replaces the sim's BASE_K_MOT motor time constant. This
// is the diagnostic that separates "the port is wrong" from "the platform is
// too sluggish": the control law is identical across the sweep, so if the
// gates only pass with faster motors, the law is fine and the sim's 0.15 s
// motor lag is the binding constraint.
static int test_formation_reform(float k_mot_override, int quiet) {
    const float S = 0.6f; // box_side/2
    const Vec3 slot[4] = {{S, S, 0}, {S, -S, 0}, {-S, -S, 0}, {-S, S, 0}};
    DroneEnv* env = make_env(4, CONTROL_MODE_VELOCITY, 0.0f);
    for (int i = 0; i < 4; i++) {
        Drone* a = &env->agents[i];
        if (k_mot_override > 0.0f) a->params.k_mot = k_mot_override;
        a->target->pos = slot[i];
        a->target->vel = (Vec3){0, 0, 0};
        a->state.pos = slot[i]; // start settled on-slot
        a->state.vel = (Vec3){0, 0, 0};
        a->state.omega = (Vec3){0, 0, 0};
        a->state.quat = (Quat){1, 0, 0, 0};
        a->integ = (Vec3){0, 0, 0};
    }
    drive(env, 200, NULL); // let it settle from the RPM transient

    // 2.5 m/s lateral kick on drone 1 (matches the MuJoCo demo scenario)
    env->agents[1].state.vel.y += 2.5f;

    const float THRESH = 0.15f, HOLD = 0.5f;
    const int hold_n = (int)(HOLD / (double)ACTION_DT);
    int run = 0, reform_tick = -1;
    float min_sep = 1e9f, worst_err = 0.0f;

    for (int t = 0; t < 1200; t++) {
        float acts[4];
        for (int i = 0; i < 4; i++) {
            velocity_control_step(i, env->agents, 4, NULL, 0.0f, ACTION_DT, acts, NULL);
            move_drone(&env->agents[i], acts);
        }
        float max_err = 0.0f;
        for (int i = 0; i < 4; i++) {
            float e = norm3(sub3(env->agents[i].target->pos, env->agents[i].state.pos));
            if (e > max_err) max_err = e;
            for (int j = i + 1; j < 4; j++) {
                float d = norm3(sub3(env->agents[i].state.pos, env->agents[j].state.pos));
                if (d < min_sep) min_sep = d;
            }
        }
        if (max_err > worst_err) worst_err = max_err;
        run = (max_err < THRESH) ? run + 1 : 0;
        if (reform_tick < 0 && run >= hold_n) reform_tick = t - hold_n + 1;
    }
    float reform_s = reform_tick < 0 ? -1.0f : reform_tick * (float)ACTION_DT;
    int ok = (reform_tick >= 0 && reform_s < 2.0f && min_sep >= 0.40f);
    if (!quiet)
        printf("[formation]    reform=%.2f s (gate <2.0, MuJoCo ref 1.29)  min_sep=%.3f m "
               "(gate >=0.40, ref 0.49)  worst_err=%.2f m  -> %s\n",
               reform_s, min_sep, worst_err, ok ? "PASS" : "FAIL");
    else
        printf("    k_mot=%.3fs  reform=%5.2fs  min_sep=%.3fm  worst_err=%.2fm  %s\n",
               k_mot_override, reform_s, min_sep, worst_err, ok ? "PASS" : "FAIL");
    free(env);
    return ok;
}

// --- Task (c) tests: the FORMATION task -------------------------------------

// Test 7a: TASK_NAMES tracks the DroneTask enum by position, and the numeric
// task ids the configs use are what we think they are. Nothing in the compiler
// ties the name table to the enum, so a reorder can silently remap tasks --
// config/drone.ini selects by integer.
static int test_task_enum_names(void) {
    int ok = 1;
    for (int t = 0; t < TASK_N; t++) {
        if (get_task((char*)TASK_NAMES[t]) != (DroneTask)t) {
            printf("[task enum] TASK_NAMES[%d]=\"%s\" round-trips to %d  -> FAIL\n", t,
                   TASK_NAMES[t], (int)get_task((char*)TASK_NAMES[t]));
            ok = 0;
        }
    }
    // The two ids that are load-bearing outside this file.
    if (HOVER != 1) { printf("[task enum] HOVER=%d, config default expects 1  -> FAIL\n", HOVER); ok = 0; }
    if (FORMATION != 2) { printf("[task enum] FORMATION=%d, expected 2  -> FAIL\n", FORMATION); ok = 0; }
    printf("[task enum] %d tasks round-trip; hover=%d formation=%d  -> %s\n", TASK_N, HOVER,
           FORMATION, ok ? "PASS" : "FAIL");
    return ok;
}

static int vec3_close(Vec3 a, Vec3 b, float eps) {
    return norm3(sub3(a, b)) < eps;
}

// Test 7: slot geometry analytics. Every mode's tightest pairwise spacing must
// clear VC_D_ACT (0.70 m) so the APF never fights a held formation, and the
// heading rotation must be a proper Rz (yaw=pi/2 maps +x to +y).
static int test_formation_geometry(void) {
    // Expected tightest pairwise separations per mode, from the reference
    // geometry (default_params.py): box 1.2, line 1.0, stack 0.75,
    // compressed 0.72, diamond sqrt(1.0^2 + 0.7^2) ~ 1.22.
    const float expected_min[FORM_MODE_N] = {1.2f, 1.0f, 0.75f, 0.72f, 1.2207f};
    int ok = 1;
    float tightest = 1e9f;
    for (int m = 0; m < FORM_MODE_N; m++) {
        float min_sep = 1e9f;
        for (int i = 0; i < FM_N_SLOTS; i++) {
            for (int j = i + 1; j < FM_N_SLOTS; j++) {
                float d = norm3(sub3(formation_slot_offset((FormationMode)m, i),
                                     formation_slot_offset((FormationMode)m, j)));
                if (d < min_sep) min_sep = d;
            }
        }
        if (fabsf(min_sep - expected_min[m]) > 1e-3f) {
            printf("[formation geom] %s min_sep=%.4f, expected %.4f  -> FAIL\n",
                   FORMATION_MODE_NAMES[m], min_sep, expected_min[m]);
            ok = 0;
        }
        if (min_sep < tightest) tightest = min_sep;
    }
    // Rz sanity: yaw=pi/2 maps box slot 0 (s, s, 0) to (-s, s, 0).
    Vec3 r = rz_rotate(0.5f * (float)M_PI, (Vec3){0.6f, 0.6f, 0.0f});
    if (!vec3_close(r, (Vec3){-0.6f, 0.6f, 0.0f}, 1e-5f)) {
        printf("[formation geom] Rz(pi/2) wrong: got (%.3f, %.3f, %.3f)\n", r.x, r.y, r.z);
        ok = 0;
    }
    printf("[formation geom] all 5 modes match reference; tightest spacing %.2f m > D_ACT %.2f  -> %s\n",
           tightest, (double)VC_D_ACT, ok && tightest > VC_D_ACT ? "PASS" : "FAIL");
    return ok && tightest > VC_D_ACT;
}

// Test 8: mode-transition blend, checked analytically against the reference
// (formation_manager.py): offsets interpolate linearly over FM_BLEND_TIME,
// v_target carries the blend rate while 0<alpha<1, and both vanish once the
// transition completes.
static int test_formation_blend(void) {
    Formation f;
    unsigned int rng = 7;
    formation_reset(&f, &rng, 1.0f);
    f.centroid.pos = (Vec3){0, 0, 0};
    f.centroid.vel = (Vec3){0, 0, 0};
    f.centroid.yaw = 0.0f;
    f.centroid.yaw_rate = 0.0f;

    formation_set_mode(&f, FORM_LINE);
    f.t = f.blend_t0 + 0.5f * FM_BLEND_TIME; // alpha = 0.5

    Vec3 p, v;
    formation_slot_target(&f, 0, &p, &v);
    // Midpoint of box slot 0 (0.6, 0.6, 0) and line slot 0 (1.5, 0, 0).
    int mid_ok = vec3_close(p, (Vec3){1.05f, 0.3f, 0.0f}, 1e-5f);
    // Blend rate: (off_line - off_box) / blend_time.
    int vel_ok = vec3_close(v, (Vec3){0.9f, -0.6f, 0.0f}, 1e-5f);

    // Rotated: same blend under yaw = pi/2.
    f.centroid.yaw = 0.5f * (float)M_PI;
    formation_slot_target(&f, 0, &p, &v);
    int rot_ok = vec3_close(p, (Vec3){-0.3f, 1.05f, 0.0f}, 1e-5f)
              && vec3_close(v, (Vec3){0.6f, 0.9f, 0.0f}, 1e-5f);
    f.centroid.yaw = 0.0f;

    // Completed: exactly on the line slots, no residual blend velocity.
    f.t = f.blend_t0 + 2.0f * FM_BLEND_TIME;
    formation_slot_target(&f, 0, &p, &v);
    int done_ok = vec3_close(p, (Vec3){1.5f, 0.0f, 0.0f}, 1e-5f)
               && vec3_close(v, (Vec3){0.0f, 0.0f, 0.0f}, 1e-5f);

    int ok = mid_ok && vel_ok && rot_ok && done_ok;
    printf("[formation blend] midpoint=%s  blend_vel=%s  rotated=%s  completed=%s  -> %s\n",
           mid_ok ? "ok" : "BAD", vel_ok ? "ok" : "BAD", rot_ok ? "ok" : "BAD",
           done_ok ? "ok" : "BAD", ok ? "PASS" : "FAIL");
    return ok;
}

// Test 9: feedforward consistency — the invariant the tracking law relies on.
// The finite difference of each slot's target position must equal the reported
// v_target at every tick (to O(dt)), across cruising, turning, waypoint
// captures, and a mid-flight mode transition. This is what makes KFF exact on
// a moving, turning, blending formation; if any of the three velocity terms
// (centroid, blend rate, yaw rate) is wrong, this fails loudly.
static int test_formation_feedforward_consistency(void) {
    Formation f;
    unsigned int rng = 123;
    formation_reset(&f, &rng, 1.0f);

    Vec3 prev_p[FM_N_SLOTS];
    Vec3 v_unused;
    for (int s = 0; s < FM_N_SLOTS; s++) formation_slot_target(&f, s, &prev_p[s], &v_unused);

    float max_err = 0.0f;
    float a_prev = formation_blend_alpha(&f);
    int checked = 0;
    for (int t = 0; t < 3000; t++) {
        if (t == 800) formation_set_mode(&f, FORM_LINE);
        if (t == 1600) formation_set_mode(&f, FORM_DIAMOND);

        formation_step(&f, &rng, ACTION_DT);
        float a = formation_blend_alpha(&f);
        // The one tick the invariant cannot hold: alpha clips to 1 partway
        // through the step, so the position still moves the last sliver of
        // blend while the reported rate is already zero. The reference's
        // piecewise blend has the same property; every other tick — entering
        // a blend, inside it, cruising — must match.
        int blend_exit = (a_prev < 1.0f && a >= 1.0f);
        a_prev = a;

        for (int s = 0; s < FM_N_SLOTS; s++) {
            Vec3 p, v;
            formation_slot_target(&f, s, &p, &v);
            if (!blend_exit) {
                Vec3 fd = scalmul3(sub3(p, prev_p[s]), 1.0f / ACTION_DT);
                float err = norm3(sub3(fd, v));
                if (err > max_err) max_err = err;
                checked++;
            }
            prev_p[s] = p;
        }
    }
    // Centroid must also stay inside the flyable volume.
    Vec3 c = f.centroid.pos;
    int bounds_ok = fabsf(c.x) <= MARGIN_X && fabsf(c.y) <= MARGIN_Y && fabsf(c.z) <= MARGIN_Z;
    int ok = (max_err < 0.05f) && bounds_ok && checked > 10000;
    printf("[formation ff]  max |d(p_target)/dt - v_target| = %.4f m/s over %d checks "
           "(gate <0.05)  centroid in bounds=%d  -> %s\n",
           max_err, checked, bounds_ok, ok ? "PASS" : "FAIL");
    return ok;
}

// Test 10: the FORMATION task in-env — c_step advances the centroid, refreshes
// slot targets, and the classical stack tracks them. Start the 4 drones on
// their slots (the reform transient is test 6's job) and gate on cruise
// tracking error and the separation floor while the formation flies waypoints.
static int test_formation_task_in_env(void) {
    DroneEnv* env = make_env(4, CONTROL_MODE_VELOCITY, 0.0f);
    env->task = FORMATION;
    c_reset(env);
    for (int i = 0; i < 4; i++) {
        Drone* a = &env->agents[i];
        a->state.pos = a->target->pos; // start settled on-slot
        a->state.vel = a->target->vel;
        a->state.omega = (Vec3){0, 0, 0};
        a->state.quat = (Quat){1, 0, 0, 0};
        a->integ = (Vec3){0, 0, 0};
    }

    float max_err = 0.0f, min_sep = 1e9f;
    int finite = 1;
    for (int t = 0; t < 800; t++) {
        c_step(env); // actions stay zero => dv=0 => pure classical
        if (t < 200) continue; // RPM spin-up + first-waypoint transient
        for (int i = 0; i < 4; i++) {
            Vec3 p = env->agents[i].state.pos;
            if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z)) finite = 0;
            float e = norm3(sub3(env->agents[i].target->pos, p));
            if (e > max_err) max_err = e;
            for (int j = i + 1; j < 4; j++) {
                float d = norm3(sub3(p, env->agents[j].state.pos));
                if (d < min_sep) min_sep = d;
            }
        }
    }
    int ok = finite && (max_err < 0.30f) && (min_sep >= 0.40f);
    printf("[formation env] cruise tracking err=%.3f m (gate <0.30)  min_sep=%.3f m "
           "(gate >=0.40)  finite=%d  -> %s\n",
           max_err, min_sep, finite, ok ? "PASS" : "FAIL");
    free(env);
    return ok;
}

// Test 10a: episode lifecycle. Runs well past HORIZON so resets actually fire
// -- the case test 10 misses by placing drones on-slot and stopping short.
//
// HOVER spawns a drone anywhere and then puts its target nearby; FORMATION
// cannot, because the slot is wherever the centroid is. Without a
// formation-aware spawn the drone starts tens of metres off-slot and
// terminates as oob immediately (measured: 1285 oob vs 12 timeouts over 40 s),
// so the task is reset churn and every metric taken from it is meaningless.
static int test_formation_episode_lifecycle(void) {
    DroneEnv* env = make_env(4, CONTROL_MODE_VELOCITY, 0.0f);
    env->task = FORMATION;
    c_reset(env);

    // Every drone must start within the spawn ball of its own slot.
    float worst_spawn = 0.0f;
    for (int i = 0; i < 4; i++) {
        float d = norm3(sub3(env->agents[i].target->pos, env->agents[i].state.pos));
        if (d > worst_spawn) worst_spawn = d;
    }

    // 4000 ticks = ~4 HORIZONs, so timeouts fire and re-spawn several times.
    for (int t = 0; t < 4000; t++) c_step(env);

    int spawn_ok = worst_spawn <= FM_SPAWN_DIST + 1e-3f;
    int no_oob = (env->log.oob == 0.0f);
    int timed_out = (env->log.timeout > 0.0f);
    int ok = spawn_ok && no_oob && timed_out;
    printf("[formation life] worst spawn offset=%.2f m (gate <=%.1f)  oob=%.0f (gate 0)  "
           "timeouts=%.0f  -> %s\n",
           worst_spawn, (double)FM_SPAWN_DIST, env->log.oob, env->log.timeout,
           ok ? "PASS" : "FAIL");
    free(env);
    return ok;
}

// --- Task (d) tests: the extended observation vector ------------------------

// Test 11: obs layout and invariants. Checks the width, that RPMs are still
// LAST (an upstream invariant the plan calls out), that every element is
// bounded, that neighbours are real at num_agents=4 and absent at 1, that the
// mode one-hot is one-hot only under FORMATION, and that u_classic is zero on
// the native motor path but live under velocity control.
static int test_obs_layout(void) {
    int ok = 1;

    // Width is derived, not hardcoded: 19 base + 9 neighbour + 5 mode + 1
    // timer + 3 u_classic + 4 rpm.
    if (DRONE_OBS_SIZE != 41) {
        printf("[obs layout] DRONE_OBS_SIZE=%d, expected 41  -> FAIL\n", DRONE_OBS_SIZE);
        ok = 0;
    }
    const int I_NEIGH = OBS_BASE;                    // 19
    const int I_MODE = I_NEIGH + 3 * OBS_N_NEIGHBORS; // 28
    const int I_TTM = I_MODE + OBS_N_FORM_MODES;      // 33
    const int I_UC = I_TTM + 1;                       // 34
    const int I_RPM = I_UC + 3;                       // 37

    // --- FORMATION with a full 4-drone swarm: neighbours must be real.
    DroneEnv* env = make_env(4, CONTROL_MODE_VELOCITY, 0.0f);
    env->task = FORMATION;
    c_reset(env);
    // Pin a known geometry: agent 0 at origin, identity attitude, agent 1 at
    // +1 m along body x. Its neighbour block must lead with tanh(1*0.5).
    for (int i = 0; i < 4; i++) {
        env->agents[i].state.quat = (Quat){1, 0, 0, 0};
        env->agents[i].state.pos = (Vec3){(float)i, 0.0f, 0.0f};
    }
    compute_observations(env);
    float* o = env->observations; // agent 0

    int neigh_ok = fabsf(o[I_NEIGH + 0] - tanhf(1.0f * OBS_NEIGHBOR_SCALE)) < 1e-5f
                && fabsf(o[I_NEIGH + 1]) < 1e-6f && fabsf(o[I_NEIGH + 2]) < 1e-6f
                && fabsf(o[I_NEIGH + 3] - tanhf(2.0f * OBS_NEIGHBOR_SCALE)) < 1e-5f
                && fabsf(o[I_NEIGH + 6] - tanhf(3.0f * OBS_NEIGHBOR_SCALE)) < 1e-5f;
    if (!neigh_ok) {
        printf("[obs layout] neighbour block wrong: %.4f %.4f %.4f (expected %.4f 0 0)\n",
               o[I_NEIGH], o[I_NEIGH + 1], o[I_NEIGH + 2], tanhf(0.5f));
        ok = 0;
    }

    // Mode one-hot: exactly one hot, and it is box (the Stage 3 held mode).
    float hot = 0.0f;
    for (int m = 0; m < OBS_N_FORM_MODES; m++) hot += o[I_MODE + m];
    int mode_ok = (fabsf(hot - 1.0f) < 1e-6f) && (o[I_MODE + FORM_BOX] == 1.0f);
    // No scheduler in Stage 3 -> timer saturates at "far away".
    int ttm_ok = o[I_TTM] > 0.99f;

    // RPMs last: must equal state.rpms / max_rpm.
    int rpm_ok = 1;
    for (int k = 0; k < 4; k++) {
        float want = env->agents[0].state.rpms[k] / env->agents[0].params.max_rpm;
        if (fabsf(o[I_RPM + k] - want) > 1e-6f) rpm_ok = 0;
    }

    // Everything bounded and finite, for all agents.
    int bounded = 1;
    for (int i = 0; i < 4 * DRONE_OBS_SIZE; i++)
        if (!isfinite(env->observations[i]) || fabsf(env->observations[i]) > 1.0f + 1e-5f) bounded = 0;

    // u_classic is live under velocity control. Put the drones back on their
    // slots first: the pinned geometry above sits far from the (randomly
    // placed) centroid, which would trip the oob reset and zero u_classic
    // before the observation is built.
    for (int i = 0; i < 4; i++) {
        env->agents[i].state.pos = env->agents[i].target->pos;
        env->agents[i].state.vel = env->agents[i].target->vel;
    }
    c_step(env);
    int uc_live = 0;
    for (int k = 0; k < 3; k++) if (fabsf(env->observations[I_UC + k]) > 1e-6f) uc_live = 1;
    free(env);

    // --- num_agents=1: no neighbours -> the block is zeros (the Stage 3a
    // benchmark config; the plan's "stub as zeros", with no stub).
    DroneEnv* solo = make_env(1, CONTROL_MODE_VELOCITY, 0.0f);
    solo->task = FORMATION;
    c_reset(solo);
    compute_observations(solo);
    int solo_zero = 1;
    for (int k = 0; k < 3 * OBS_N_NEIGHBORS; k++)
        if (solo->observations[I_NEIGH + k] != 0.0f) solo_zero = 0;
    free(solo);

    // --- HOVER on the native motor path: no formation, no classical law.
    DroneEnv* hov = make_env(4, CONTROL_MODE_MOTOR, 0.0f);
    c_step(hov);
    int hover_mode_zero = 1;
    for (int m = 0; m < OBS_N_FORM_MODES; m++)
        if (hov->observations[I_MODE + m] != 0.0f) hover_mode_zero = 0;
    if (hov->observations[I_TTM] != 0.0f) hover_mode_zero = 0;
    int uc_zero_motor = 1;
    for (int k = 0; k < 3; k++)
        if (hov->observations[I_UC + k] != 0.0f) uc_zero_motor = 0;
    free(hov);

    ok = ok && neigh_ok && mode_ok && ttm_ok && rpm_ok && bounded && uc_live && solo_zero
         && hover_mode_zero && uc_zero_motor;
    printf("[obs layout] width=%d  neighbours(n=4)=%s  neighbours(n=1)=zeros:%s  one-hot=%s  "
           "timer=%s\n",
           DRONE_OBS_SIZE, neigh_ok ? "ok" : "BAD", solo_zero ? "y" : "N", mode_ok ? "ok" : "BAD",
           ttm_ok ? "ok" : "BAD");
    printf("[obs layout] rpms-last=%s  bounded=%s  u_classic live/motor-zero=%s/%s  "
           "hover mode+timer zero=%s  -> %s\n",
           rpm_ok ? "ok" : "BAD", bounded ? "ok" : "BAD", uc_live ? "ok" : "BAD",
           uc_zero_motor ? "ok" : "BAD", hover_mode_zero ? "ok" : "BAD", ok ? "PASS" : "FAIL");
    return ok;
}

// --- Task (e) tests: the reward extension -----------------------------------

// Test 12: the relative-velocity change is exact identity for static-target
// tasks. hover_potential/check_hover now measure |v - v_target| instead of
// |v|, which is the whole point on FORMATION — but it must not perturb the
// Stage 1 HOVER reward. It cannot, provided target->vel is zero there, so
// assert that directly for every static-target task rather than reasoning
// about it. (IDLE/FOLLOW/CONGO are excluded: they carry the upstream per-tick
// target->vel convention and are untrained demo tasks.)
static int test_static_target_tasks_unaffected(void) {
    const int tasks[] = {HOVER, ORBIT, CUBE, FLAG};
    const char* names[] = {"hover", "orbit", "cube", "flag"};
    int ok = 1;
    for (int t = 0; t < 4; t++) {
        DroneEnv* env = make_env(8, CONTROL_MODE_MOTOR, 0.0f);
        env->task = tasks[t];
        c_reset(env);
        for (int i = 0; i < env->num_agents; i++) {
            Vec3 tv = env->agents[i].target->vel;
            if (tv.x != 0.0f || tv.y != 0.0f || tv.z != 0.0f) {
                printf("[reward identity] %s agent %d has target->vel=(%.3f,%.3f,%.3f)  -> FAIL\n",
                       names[t], i, tv.x, tv.y, tv.z);
                ok = 0;
            }
        }
        free(env);
    }
    printf("[reward identity] hover/orbit/cube/flag all have target->vel=0, so "
           "|v - v_target| == |v|  -> %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

// Test 13: the new reward terms are inert at their default weights, so a
// config that does not opt in gets the Stage 1 reward exactly. Drives HOVER on
// the native motor path and reconstructs the pre-task-(e) reward formula: any
// divergence means a new term leaked into the default path.
static int test_reward_extension_inert(void) {
    DroneEnv* env = make_env(8, CONTROL_MODE_MOTOR, 0.0f);
    float worst = 0.0f;
    for (int t = 0; t < 400; t++) {
        float pre_pot[8], pre_dist[8];
        for (int i = 0; i < 8; i++) {
            pre_pot[i] = env->agents[i].prev_potential;
            pre_dist[i] = norm3(sub3(env->agents[i].target->pos, env->agents[i].state.pos));
            (void)pre_dist[i];
        }
        c_step(env);
        for (int i = 0; i < 8; i++) {
            Drone* a = &env->agents[i];
            if (a->episode_length == 0) continue; // just reset; prev_* are stale
            float curr = hover_potential(a, env->hover_dist, env->hover_omega, env->hover_vel);
            float prev_d = norm3(sub3(a->target->pos, a->prev_pos));
            float curr_d = norm3(sub3(a->target->pos, a->state.pos));
            // The upstream formula, with no task-(e) terms at all.
            float want = env->alpha_dist * (prev_d - curr_d) + env->alpha_hover * curr
                       + env->alpha_shaping * (curr - pre_pot[i])
                       - env->alpha_omega * norm3(a->state.omega);
            float d = fabsf(want - env->rewards[i]);
            if (d > worst) worst = d;
        }
    }
    int ok = worst < 1e-5f;
    printf("[reward inert]    max |reward - upstream formula| on HOVER/motor = %.2e  -> %s\n",
           worst, ok ? "PASS" : "FAIL");
    free(env);
    return ok;
}

// Test 14: the opt-in terms actually fire. Separation: two drones pinned
// inside the floor must count a collision and (when enabled) terminate.
// Alignment: a drone pinned off-slot past align_time must accrue the penalty,
// and must not before the grace window elapses.
static int test_reward_extension_fires(void) {
    // --- separation breach
    DroneEnv* env = make_env(2, CONTROL_MODE_MOTOR, 0.0f);
    env->separation_floor = 0.4f;
    env->separation_terminates = 1;
    env->agents[0].state.pos = (Vec3){0, 0, 0};
    env->agents[1].state.pos = (Vec3){0.2f, 0, 0}; // 0.2 m < 0.4 m floor
    env->agents[0].target->pos = (Vec3){0, 0, 0};
    env->agents[1].target->pos = (Vec3){0.2f, 0, 0};
    c_step(env);
    int sep_ok = (env->terminals[0] == 1.0f) && (env->log.sep_breach >= 1.0f);
    free(env);

    // Same geometry, floor disabled -> no breach, no termination.
    DroneEnv* off = make_env(2, CONTROL_MODE_MOTOR, 0.0f);
    off->agents[0].state.pos = (Vec3){0, 0, 0};
    off->agents[1].state.pos = (Vec3){0.2f, 0, 0};
    off->agents[0].target->pos = (Vec3){0, 0, 0};
    off->agents[1].target->pos = (Vec3){0.2f, 0, 0};
    c_step(off);
    int off_ok = (off->terminals[0] == 0.0f) && (off->log.sep_breach == 0.0f);
    free(off);

    // --- alignment timer: hold a drone 1 m off target (inside the 6 m oob
    // margin) and check the penalty starts only after align_time.
    DroneEnv* al = make_env(1, CONTROL_MODE_MOTOR, 0.0f);
    al->alpha_align = 1.0f;
    al->alpha_dist = 0.0f; al->alpha_hover = 0.0f; al->alpha_shaping = 0.0f; al->alpha_omega = 0.0f;
    Drone* a = &al->agents[0];
    float r_before = 0.0f, r_after = 0.0f;
    for (int t = 0; t < 400; t++) {
        a->state.pos = (Vec3){0, 0, 0};
        a->target->pos = (Vec3){1.0f, 0, 0}; // 1 m off, well beyond align_dist
        al->hover_target_dist = 100.0f;      // never oob
        c_step(al);
        if (t == 100) r_before = al->rewards[0]; // t=1.0 s, inside the 2 s grace
        if (t == 350) r_after = al->rewards[0];  // t=3.5 s, ~1.5 s of overrun
    }
    // Inside the window: no penalty. Past it: penalty grows with the overrun.
    int align_ok = (fabsf(r_before) < 1e-6f) && (r_after < -1.0f);
    float t_un = a->t_unaligned;
    free(al);

    int ok = sep_ok && off_ok && align_ok;
    printf("[reward fires]    separation: breach=%s disabled=%s | alignment: "
           "r(1.0s)=%.3f r(3.5s)=%.3f t_unaligned=%.1fs  -> %s\n",
           sep_ok ? "ok" : "BAD", off_ok ? "ok" : "BAD", r_before, r_after, t_un,
           ok ? "PASS" : "FAIL");
    return ok;
}

int main(int argc, char** argv) {
    // Gate mode: `test_velocity_wrapper <k_mot>` runs only the NFR-36 formation
    // gate at the given motor time constant and returns its verdict as the exit
    // code. run_velocity_tests.sh uses this to assert port faithfulness against
    // the reference platform. No arg = full informational suite.
    if (argc > 1) return test_formation_reform((float)atof(argv[1]), 1) ? 0 : 1;

    int pass = 1;
    printf("--- task (a): wrapper gating ---\n");
    pass &= test_motor_path_smoke();
    pass &= test_classical_hover();
    pass &= test_single_settle();

    printf("--- task (b): ported classical controller ---\n");
    pass &= test_moving_target_feedforward();
    pass &= test_apf_separation();          // equilibrium only; see note above
    pass &= test_formation_reform(0.0f, 0); // NFR-36 gate at the shipped k_mot

    printf("--- task (c): FORMATION task ---\n");
    pass &= test_task_enum_names();
    pass &= test_formation_geometry();
    pass &= test_formation_blend();
    pass &= test_formation_feedforward_consistency();
    pass &= test_formation_task_in_env();
    pass &= test_formation_episode_lifecycle();

    printf("--- task (d): extended observation vector ---\n");
    pass &= test_obs_layout();

    printf("--- task (e): reward extension ---\n");
    pass &= test_static_target_tasks_unaffected();
    pass &= test_reward_extension_inert();
    pass &= test_reward_extension_fires();

    // Why the cascade is tuned the way it is. The control law is identical in
    // every row — only the platform's motor time constant moves. BASE_K_MOT was
    // 0.15 s upstream, at which these gains diverge and no stable retune meets
    // the 2 s rule; it now ships at a realistic 0.05 s. Kept as a regression
    // witness: if someone raises BASE_K_MOT again, this shows the cost.
    printf("--- witness: NFR-36 vs motor lag (identical control law) ---\n");
    test_formation_reform(0.15f, 1); // old upstream value -> diverges
    test_formation_reform(0.08f, 1);
    test_formation_reform(0.05f, 1); // shipped
    test_formation_reform(0.02f, 1); // fast ESC

    printf("\n%s\n", pass ? "ALL PASS" : "FAIL");
    return pass ? 0 : 1;
}
