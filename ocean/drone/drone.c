#include "drone.h"
#include "puffernet.h"
#include "render.h"
#include <time.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#ifdef __EMSCRIPTEN__
typedef struct {
    DroneEnv* env;
    PufferNet* net;
    Weights* weights;
} WebRenderArgs;

void emscriptenStep(void* e) {
    WebRenderArgs* args = (WebRenderArgs*)e;
    DroneEnv* env = args->env;
    PufferNet* net = args->net;

    if (net != NULL) forward_puffernet(net, env->observations, env->actions);
    c_step(env);
    c_render(env);
}

WebRenderArgs* web_args = NULL;
#endif

// Read an env-var override, or fall back to a default.
static const char* env_or(const char* name, const char* fallback) {
    const char* v = getenv(name);
    return (v != NULL && v[0] != '\0') ? v : fallback;
}

int main() {
    srand(time(NULL));

    DroneEnv* env = calloc(1, sizeof(DroneEnv));

    // Defaults to the Stage 3a classical FORMATION baseline, because that is
    // the one that runs correctly out of the box: it needs no policy at all
    // (k_res = 0 discards the network's actions), whereas the bundled
    // resources/drone/drone_weights.bin predates the Stage 3 observation
    // extension and is sized for the old 23-float vector. Override with:
    //
    //   DRONE_TASK=hover DRONE_CONTROL_MODE=0 ./drone   # upstream demo
    //   DRONE_TASK=formation DRONE_K_RES=0.5 ./drone    # classical + residual
    //
    // DRONE_TASK takes any name from TASK_NAMES (tasks.h).
    env->task = get_task((char*)env_or("DRONE_TASK", "formation"));
    env->control_mode = atoi(env_or("DRONE_CONTROL_MODE", "1"));
    env->k_res = (float)atof(env_or("DRONE_K_RES", "0"));

    // One env holds one formation, so FORMATION caps at the swarm size;
    // c_reset enforces this. The other tasks keep upstream's 16.
    env->num_agents = (env->task == FORMATION) ? FM_N_SLOTS : 16;

    env->max_rings = 10;
    env->alpha_dist = 0.782192f;
    env->alpha_hover = 0.071445f;
    env->alpha_shaping = 3.9754f;
    env->alpha_omega = 0.00135588f;
    env->hover_target_dist = 5.0f;
    env->hover_dist = 0.1f;
    env->hover_omega = 0.1f;
    env->hover_vel = 0.1f;
    env->align_dist = 0.15f;
    env->align_time = 2.0f;
    env->separation_floor = (env->task == FORMATION) ? 0.4f : 0.0f;
    env->separation_terminates = 0;

    // Must track DRONE_OBS_SIZE: compute_observations writes that many floats
    // per agent, so a stale literal here overflows the buffer.
    size_t obs_size = DRONE_OBS_SIZE;
    env->observations = (float*)calloc(env->num_agents * obs_size, sizeof(float));
    env->actions = (float*)calloc(env->num_agents * 4, sizeof(float));
    env->rewards = (float*)calloc(env->num_agents, sizeof(float));
    env->terminals = (float*)calloc(env->num_agents, sizeof(float));

    // With the classical controller driving and no residual, the policy's
    // output is discarded — so skip the network entirely rather than feed it
    // weights whose input dim no longer matches the observation.
    bool policy_used = !(env->control_mode == CONTROL_MODE_VELOCITY && env->k_res == 0.0f);
    Weights* weights = NULL;
    PufferNet* net = NULL;
    if (policy_used) {
        weights = load_weights("resources/drone/drone_weights.bin");
        int logit_sizes[4] = {1, 1, 1, 1};
        // make_puffernet(weights, num_agents, obs_size, hidden_size, num_layers, logit_sizes, num_actions)
        net = make_puffernet(weights, env->num_agents, obs_size, 128, 3, logit_sizes, 4);
        printf("drone: task=%s control_mode=%d k_res=%.2f — policy active.\n"
               "  NOTE: resources/drone/drone_weights.bin predates the Stage 3 obs\n"
               "  extension (23 -> %d floats) and will not produce sane actions.\n",
               TASK_NAMES[env->task], env->control_mode, (double)env->k_res, DRONE_OBS_SIZE);
    } else {
        printf("drone: task=%s — pure classical (k_res=0), no policy needed.\n",
               TASK_NAMES[env->task]);
    }

    init(env);
    c_reset(env);

#ifdef __EMSCRIPTEN__
    WebRenderArgs* args = calloc(1, sizeof(WebRenderArgs));
    args->env = env;
    args->net = net;
    args->weights = weights;
    web_args = args;

    emscripten_set_main_loop_arg(emscriptenStep, args, 0, true);
#else
    c_render(env);
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        if (net != NULL) forward_puffernet(net, env->observations, env->actions);
        c_step(env);
        c_render(env);
    }

    c_close(env);
    if (net != NULL) free_puffernet(net);
    free(weights);
    free(env->observations);
    free(env->actions);
    free(env->rewards);
    free(env->terminals);
    free(env);
#endif

    return 0;
}
