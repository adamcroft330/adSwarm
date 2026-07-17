// Originally made by Sam Turner and Finlay Sanders, 2025.
// Included in pufferlib under the original project's MIT license.
// https://github.com/tensaur/drone

#pragma once

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "dronelib.h"
#include "tasks.h"
#include "velocity_controller.h"

#define HORIZON 1024

typedef struct Client Client;
typedef struct DroneEnv DroneEnv;

struct DroneEnv {
    Log log;
    float* observations;
    float* actions;
    float* rewards;
    float* terminals;
    int num_agents;
    unsigned int rng;

    int tick;
    DroneTask task;
    Drone* agents;

    int max_rings;
    Target* ring_buffer;

    Client* client;

    // reward scaling
    float alpha_dist;
    float alpha_hover;
    float alpha_shaping;
    float alpha_omega;

    // Stage 3 task (e) reward extension. All default to 0 / disabled, so the
    // Stage 1 HOVER reward is bit-for-bit unchanged unless a run opts in.
    float alpha_jerk;  // penalty on velocity-setpoint deltas [per (m/s)]
    float alpha_align; // penalty per second spent unaligned beyond align_time
    float align_dist;  // "on slot" tolerance [m]
    float align_time;  // grace before the unaligned penalty starts [s] (NFR-36: 2.0)
    // Inter-drone separation floor [m]. 0 disables the check entirely — which
    // is what HOVER wants: it packs 64 independent drones into one env with
    // unrelated targets, so they routinely pass close and a breach there means
    // nothing. FORMATION sets 0.4 (FR-15).
    float separation_floor;
    int separation_terminates; // 1: a breach ends the episode; 0: count only

    // hover task parameters
    float hover_target_dist;
    float hover_dist;
    float hover_omega;
    float hover_vel;

    // Stirling velocity-setpoint stack (velocity_controller.h).
    // control_mode=0: native motor actions (Stage 1 path, default).
    // control_mode=1: classical velocity control + k_res-scaled residual.
    int control_mode;
    float k_res;

    // Formation task (tasks.h): shared virtual centroid + slot geometry.
    // Advanced once per tick in c_step; unused by the other tasks.
    Formation formation;
};

void init(DroneEnv* env) {
    env->agents = (Drone*)calloc(env->num_agents, sizeof(Drone));
    env->ring_buffer = (Target*)calloc(env->max_rings, sizeof(Target));

    for (int i = 0; i < env->num_agents; i++) {
        env->agents[i].target = (Target*)calloc(1, sizeof(Target));
        env->agents[i].buffer_idx = 0;
    }

    env->log = (Log){0};
    env->tick = 0;
}

void add_log(DroneEnv* env, int idx, bool oob, bool timeout, bool breach) {
    Drone* agent = &env->agents[idx];

    env->log.episode_return += agent->episode_return;
    env->log.episode_length += agent->episode_length;
    env->log.collisions += agent->collisions;

    if (oob) env->log.oob += 1.0f;
    if (timeout) env->log.timeout += 1.0f;
    if (breach) env->log.sep_breach += 1.0f;

    env->log.score += agent->hover_score;
    env->log.perf += agent->hover_ema;
    env->log.rings_passed += agent->rings_passed;
    env->log.ema_dist += agent->ema_dist;
    env->log.ema_vel += agent->ema_vel;
    env->log.ema_omega += agent->ema_omega;

    env->log.n += 1.0f;

    agent->episode_length = 0;
    agent->episode_return = 0.0f;
    agent->collisions = 0.0f;
    agent->score = 0.0f;
    agent->rings_passed = 0.0f;
}

void compute_observations(DroneEnv* env) {
    // Formation context is shared by every agent in the env; -1 tells the
    // builder this is not the FORMATION task (mode one-hot / timer read zero).
    int mode = (env->task == FORMATION) ? (int)env->formation.mode : -1;
    float ttm = (env->task == FORMATION) ? formation_time_to_mode_change(&env->formation) : 0.0f;

    for (int i = 0; i < env->num_agents; i++) {
        compute_drone_observations(&env->agents[i], env->agents, env->num_agents, i, mode, ttm,
                                   env->observations + i * DRONE_OBS_SIZE);
    }
}

void reset_agent(DroneEnv* env, Drone* agent, int idx) {
    agent->episode_return = 0.0f;
    agent->episode_length = 0;
    agent->collisions = 0.0f;
    agent->rings_passed = 0;
    agent->score = 0.0f;
    agent->hover_score = 0.0f;
    agent->hover_ema = 0.0f;
    agent->ema_dist = 0.0f;
    agent->ema_vel = 0.0f;
    agent->ema_omega = 0.0f;

    agent->buffer = env->ring_buffer;
    agent->buffer_size = env->max_rings;

    init_drone(agent, &env->rng, 0.05f);

    agent->state.pos =
        (Vec3){rndf(-MARGIN_X, MARGIN_X, &env->rng), rndf(-MARGIN_Y, MARGIN_Y, &env->rng), rndf(-MARGIN_Z, MARGIN_Z, &env->rng)};

    if (env->task == FORMATION) {
        // Overrides the random grid spawn above: a FORMATION drone must start
        // at its slot, which is wherever the centroid currently is.
        formation_spawn_state(&env->formation, idx, &env->rng, &agent->state.pos, &agent->state.vel);
    }

    if (env->task == RACE) {
        while (norm3(sub3(agent->state.pos, env->ring_buffer[0].pos)) < 2.0f * RING_RADIUS) {
            agent->state.pos = (Vec3){rndf(-MARGIN_X, MARGIN_X, &env->rng), rndf(-MARGIN_Y, MARGIN_Y, &env->rng),
                                      rndf(-MARGIN_Z, MARGIN_Z, &env->rng)};
        }
    }

    agent->prev_pos = agent->state.pos;
    agent->prev_potential = hover_potential(agent, env->hover_dist, env->hover_omega, env->hover_vel);
}

void c_reset(DroneEnv* env) {
    if (env->task == RACE) {
        reset_rings(&env->rng, env->ring_buffer, env->max_rings);
    }
    if (env->task == FORMATION) {
        // The swarm is 4 drones (tech doc §2) and an env holds exactly one
        // formation, so num_agents > FM_N_SLOTS would alias several drones
        // onto the same slot — coincident targets that the APF then fights.
        // Throughput does not need packing here: vecenv spawns envs until
        // total_agents is reached, so num_drones=4 gives 4x more envs at the
        // same agent count. Fail loudly rather than train on a broken task.
        if (env->num_agents > FM_N_SLOTS) {
            fprintf(stderr,
                    "drone: task=formation requires num_drones <= %d (the swarm size), got %d.\n"
                    "       Each env holds one formation; extra drones would share slots.\n"
                    "       Set num_drones=4 in config — total_agents still sets throughput.\n",
                    FM_N_SLOTS, env->num_agents);
            exit(1);
        }
        formation_reset(&env->formation, &env->rng, FM_CRUISE_SPEED);
    }

    for (int i = 0; i < env->num_agents; i++) {
        Drone* agent = &env->agents[i];
        reset_agent(env, agent, i);
        set_target(&env->rng, env->task, env->agents, i, env->num_agents, env->hover_target_dist, &env->formation);
    }

    compute_observations(env);
}

void c_step(DroneEnv* env) {
    env->tick = (env->tick + 1) % HORIZON;

    // Advance the formation centroid once per tick, then refresh every slot's
    // world-frame target (and its feedforward velocity) before control runs.
    // The moving target is what the classical tracker's KFF term consumes.
    if (env->task == FORMATION) {
        formation_step(&env->formation, &env->rng, ACTION_DT);
        for (int i = 0; i < env->num_agents; i++) {
            set_target_formation(&env->formation, &env->agents[i], i);
        }
    }

    for (int i = 0; i < env->num_agents; i++) {
        Drone* agent = &env->agents[i];

        agent->prev_pos = agent->state.pos;
        if (env->control_mode == CONTROL_MODE_VELOCITY) {
            // Policy actions become the 3-float residual dv (4th unused);
            // k_res=0 gives the pure classical dv=0 benchmark.
            float motor_actions[4];
            velocity_control_step(i, env->agents, env->num_agents, &env->actions[4 * i],
                                  env->k_res, ACTION_DT, motor_actions, NULL);
            move_drone(agent, motor_actions);
        } else {
            move_drone(agent, &env->actions[4 * i]);
        }
        agent->episode_length++;

        bool oob = norm3(sub3(agent->target->pos, agent->state.pos)) > (env->hover_target_dist + 1.0f);
        bool timeout = (agent->episode_length >= HORIZON);

        float curr = hover_potential(agent, env->hover_dist, env->hover_omega, env->hover_vel);
        float prev_dist = norm3(sub3(agent->target->pos, agent->prev_pos));
        float curr_dist = norm3(sub3(agent->target->pos, agent->state.pos));
        float omega = norm3(agent->state.omega);

        // Inter-drone separation (FR-15). Skipped entirely at floor 0, which
        // keeps HOVER's cost and behaviour untouched — it is O(n^2), and its
        // 64 independent drones breach constantly and meaninglessly.
        bool breach = false;
        if (env->separation_floor > 0.0f) {
            for (int j = 0; j < env->num_agents; j++) {
                if (j == i) continue;
                if (norm3(sub3(agent->state.pos, env->agents[j].state.pos)) < env->separation_floor) {
                    breach = true;
                    break;
                }
            }
            if (breach) agent->collisions += 1.0f;
        }

        // NFR-36's 2 s rule as a graduated penalty: nothing while aligned or
        // inside the grace window, then linear in the overrun.
        if (curr_dist <= env->align_dist) agent->t_unaligned = 0.0f;
        else agent->t_unaligned += ACTION_DT;
        float unaligned = fmaxf(0.0f, agent->t_unaligned - env->align_time);

        // Jerk on the commanded setpoint. Identically zero on the motor path,
        // where v_cmd is never written.
        float jerk = norm3(sub3(agent->v_cmd, agent->prev_v_cmd));

        float reward = env->alpha_dist * (prev_dist - curr_dist)
                     + env->alpha_hover * curr
                     + env->alpha_shaping * (curr - agent->prev_potential)
                     - env->alpha_omega * omega
                     - env->alpha_jerk * jerk
                     - env->alpha_align * unaligned;

        agent->prev_potential = curr;

        float h = check_hover(agent, env->hover_dist, env->hover_omega, env->hover_vel);
        agent->hover_score += h;
        agent->hover_ema = (1.0f - 0.02f) * agent->hover_ema + 0.02f * h;
        agent->ema_dist = 0.99f * agent->ema_dist + 0.01f * curr_dist;
        agent->ema_vel = 0.99f * agent->ema_vel + 0.01f * norm3(agent->state.vel);
        agent->ema_omega = 0.99f * agent->ema_omega + 0.01f * omega;
        agent->episode_return += reward;
        env->rewards[i] = reward;

        bool reset = oob || timeout || (breach && env->separation_terminates);
        env->terminals[i] = reset ? 1.0f : 0.0f;

        if (reset) {
            add_log(env, i, oob, timeout, breach);
            reset_agent(env, agent, i);
            set_target(&env->rng, env->task, env->agents, i, env->num_agents, env->hover_target_dist, &env->formation);
        }
    }

    compute_observations(env);
}

void c_close_client(Client* client);

void c_close(DroneEnv* env) {
    for (int i = 0; i < env->num_agents; i++) {
        free(env->agents[i].target);
    }

    free(env->agents);
    free(env->ring_buffer);

    if (env->client != NULL) {
        c_close_client(env->client);
    }
}
