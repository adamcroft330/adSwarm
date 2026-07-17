// Stage 3a classical benchmark (stage3_plan.md task f).
//
//   bash stirling/tests/run_stage3a_bench.sh
//
// Records the FORMATION floor the Stage 3b residual must beat: pure classical
// control (k_res = 0, so policy actions are ignored entirely — no policy and
// no GPU are involved, per the plan's "no RL training in this run").
//
// Metrics mirror what training reports, so the 3a floor and the 3b residual
// are directly comparable: the aggregation below is the same add_log() /
// my_log() path the Modal runs use, driven over many envs and several
// horizons instead of by a learner.
//
// Deviation from the plan, deliberate: the plan specifies num_agents = 1 with
// stubbed neighbours. The swarm is 4 (tech doc §2), neighbours are real at 4
// (see the task-d entry in progress_log.md), and 3b will train at 4 — so the
// floor must be measured at 4 or it is not the same task. n=1 is reported too,
// as the no-APF, no-neighbour reference point.
//
// separation_floor is set but separation_terminates is 0: breaches are counted
// without altering the dynamics being measured.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "drone.h"

// render.h provides this in the real build; stub it for the headless bench.
void c_close_client(Client* client) { (void)client; }

#define N_ENVS 256
#define HORIZONS 4
#define TICKS (HORIZON * HORIZONS)

typedef struct {
    double score, perf, episode_return, episode_length;
    double ema_dist, ema_vel, ema_omega;
    double oob, timeout, collisions, sep_breach, n;
    double track_err, min_sep, worst_err;
} Bench;

static DroneEnv* make_env(int idx, int num_drones) {
    DroneEnv* env = calloc(1, sizeof(DroneEnv));
    env->num_agents = num_drones;
    env->max_rings = 10;
    env->task = FORMATION;
    // config/drone.ini [env] values
    env->alpha_dist = 0.782192f;
    env->alpha_hover = 0.071445f;
    env->alpha_shaping = 3.9754f;
    env->alpha_omega = 0.00135588f;
    env->hover_target_dist = 5.0f;
    env->hover_dist = 0.1f;
    env->hover_omega = 0.1f;
    env->hover_vel = 0.1f;
    env->alpha_jerk = 0.0f;
    env->alpha_align = 0.0f;
    env->align_dist = 0.15f;
    env->align_time = 2.0f;
    env->separation_floor = 0.4f;  // FR-15: count breaches...
    env->separation_terminates = 0; // ...without perturbing what we measure
    // Box-only, per the competition brief: a box must be held throughout, and
    // deviations are legal only to clear an obstacle. The env has no obstacles
    // until Stage 4, so nothing may legally leave box — this is the task.
    env->formation_modes = 0;
    env->control_mode = CONTROL_MODE_VELOCITY;
    env->k_res = 0.0f; // pure classical: actions ignored
    env->rng = idx;    // vecenv seeds envs by index
    env->observations = calloc(num_drones * DRONE_OBS_SIZE, sizeof(float));
    env->actions = calloc(num_drones * 4, sizeof(float));
    env->rewards = calloc(num_drones, sizeof(float));
    env->terminals = calloc(num_drones, sizeof(float));
    init(env);
    c_reset(env);
    return env;
}

static void free_env(DroneEnv* env) {
    free(env->observations);
    free(env->actions);
    free(env->rewards);
    free(env->terminals);
    c_close(env);
    free(env);
}

static Bench run(int num_drones) {
    Bench b = {0};
    b.min_sep = 1e9;
    double steps = 0;

    for (int e = 0; e < N_ENVS; e++) {
        DroneEnv* env = make_env(e, num_drones);
        for (int t = 0; t < TICKS; t++) {
            c_step(env); // actions stay zero; k_res=0 ignores them regardless
            for (int i = 0; i < num_drones; i++) {
                Drone* a = &env->agents[i];
                float err = norm3(sub3(a->target->pos, a->state.pos));
                b.track_err += err;
                if (err > b.worst_err) b.worst_err = err;
                for (int j = i + 1; j < num_drones; j++) {
                    float d = norm3(sub3(a->state.pos, env->agents[j].state.pos));
                    if (d < b.min_sep) b.min_sep = d;
                }
                steps++;
            }
        }
        Log* l = &env->log;
        b.score += l->score;
        b.perf += l->perf;
        b.episode_return += l->episode_return;
        b.episode_length += l->episode_length;
        b.ema_dist += l->ema_dist;
        b.ema_vel += l->ema_vel;
        b.ema_omega += l->ema_omega;
        b.oob += l->oob;
        b.timeout += l->timeout;
        b.collisions += l->collisions;
        b.sep_breach += l->sep_breach;
        b.n += l->n;
        free_env(env);
    }

    b.track_err /= steps;
    if (b.n > 0) {
        b.score /= b.n;
        b.perf /= b.n;
        b.episode_return /= b.n;
        b.episode_length /= b.n;
        b.ema_dist /= b.n;
        b.ema_vel /= b.n;
        b.ema_omega /= b.n;
    }
    return b;
}

static void report(const char* label, Bench b, int num_drones) {
    printf("--- %s (num_drones=%d, %d envs x %d ticks) ---\n", label, num_drones, N_ENVS, TICKS);
    printf("  episodes            %.0f\n", b.n);
    printf("  score               %.3f\n", b.score);
    printf("  perf                %.4f\n", b.perf);
    printf("  episode_return      %.3f\n", b.episode_return);
    printf("  episode_length      %.1f  (HORIZON %d)\n", b.episode_length, HORIZON);
    printf("  ema_dist            %.4f\n", b.ema_dist);
    printf("  ema_vel             %.4f\n", b.ema_vel);
    printf("  ema_omega           %.4f\n", b.ema_omega);
    printf("  oob                 %.0f  (%.2f%% of episodes)\n", b.oob,
           b.n > 0 ? 100.0 * b.oob / b.n : 0.0);
    printf("  timeout             %.0f\n", b.timeout);
    printf("  collisions (ticks)  %.0f\n", b.collisions);
    printf("  sep_breach          %.0f\n", b.sep_breach);
    printf("  mean tracking err   %.4f m\n", b.track_err);
    printf("  worst tracking err  %.4f m\n", b.worst_err);
    if (num_drones > 1)
        printf("  min separation      %.4f m  (floor %.2f)\n", b.min_sep, 0.40);
    printf("\n");
}

int main(void) {
    printf("Stage 3a — classical FORMATION floor (k_res=0, pure classical)\n");
    printf("The floor the Stage 3b residual must beat. No policy, no GPU.\n\n");

    Bench b4 = run(4);
    report("SWARM: the configuration 3b trains on", b4, 4);

    Bench b1 = run(1);
    report("SOLO: reference point, no neighbours and no APF interaction", b1, 1);

    printf("Compare 3b against the SWARM row: same task, same num_drones.\n");
    return 0;
}
