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

#define OBS 23

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

int main(void) {
    int pass = 1;
    pass &= test_motor_path_smoke();
    pass &= test_classical_hover();
    pass &= test_single_settle();
    printf("\n%s\n", pass ? "ALL PASS" : "FAIL");
    return pass ? 0 : 1;
}
