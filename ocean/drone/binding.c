#include "drone.h"
#include "render.h"

#define OBS_SIZE DRONE_OBS_SIZE // 41; layout documented in dronelib.h
#define NUM_ATNS 4
#define ACT_SIZES {1, 1, 1, 1}
#define OBS_TENSOR_T FloatTensor

#define Env DroneEnv
#include "vecenv.h"

void my_init(Env* env, Dict* kwargs) {
    env->num_agents = (int)dict_get(kwargs, "num_drones")->value;
    env->task = (int)dict_get(kwargs, "task")->value;
    env->max_rings = (int)dict_get(kwargs, "max_rings")->value;
    env->alpha_dist = dict_get(kwargs, "alpha_dist")->value;
    env->alpha_hover = dict_get(kwargs, "alpha_hover")->value;
    env->alpha_shaping = dict_get(kwargs, "alpha_shaping")->value;
    env->alpha_omega = dict_get(kwargs, "alpha_omega")->value;
    env->hover_target_dist = dict_get(kwargs, "hover_target_dist")->value;
    env->hover_dist = dict_get(kwargs, "hover_dist")->value;
    env->hover_omega = dict_get(kwargs, "hover_omega")->value;
    env->hover_vel = dict_get(kwargs, "hover_vel")->value;

    // Stirling velocity-setpoint stack: optional kwargs so existing configs
    // (Stage 1) keep the native motor path without any ini change.
    DictItem* control_mode = dict_get_unsafe(kwargs, "control_mode");
    env->control_mode = control_mode ? (int)control_mode->value : CONTROL_MODE_MOTOR;
    DictItem* k_res = dict_get_unsafe(kwargs, "k_res");
    env->k_res = k_res ? (float)k_res->value : 0.0f;

    // Task (e) reward extension. Optional like the two above, and every
    // default is inert, so a config that omits them gets the Stage 1 reward.
    DictItem* alpha_jerk = dict_get_unsafe(kwargs, "alpha_jerk");
    env->alpha_jerk = alpha_jerk ? (float)alpha_jerk->value : 0.0f;
    DictItem* alpha_align = dict_get_unsafe(kwargs, "alpha_align");
    env->alpha_align = alpha_align ? (float)alpha_align->value : 0.0f;
    DictItem* align_dist = dict_get_unsafe(kwargs, "align_dist");
    env->align_dist = align_dist ? (float)align_dist->value : 0.15f;
    DictItem* align_time = dict_get_unsafe(kwargs, "align_time");
    env->align_time = align_time ? (float)align_time->value : 2.0f;
    DictItem* separation_floor = dict_get_unsafe(kwargs, "separation_floor");
    env->separation_floor = separation_floor ? (float)separation_floor->value : 0.0f;
    DictItem* separation_terminates = dict_get_unsafe(kwargs, "separation_terminates");
    env->separation_terminates = separation_terminates ? (int)separation_terminates->value : 0;
    DictItem* formation_modes = dict_get_unsafe(kwargs, "formation_modes");
    env->formation_modes = formation_modes ? (int)formation_modes->value : 0;
    DictItem* formation_speed = dict_get_unsafe(kwargs, "formation_speed");
    env->formation_speed = formation_speed ? (float)formation_speed->value : FM_CRUISE_SPEED;
    init(env);
}

void my_log(Log* log, Dict* out) {
    dict_set(out, "perf", log->perf);
    dict_set(out, "score", log->score);
    dict_set(out, "rings_passed", log->rings_passed);
    dict_set(out, "ring_collisions", log->ring_collision);
    dict_set(out, "collisions", log->collisions);
    dict_set(out, "oob", log->oob);
    dict_set(out, "sep_breach", log->sep_breach);
    dict_set(out, "timeout", log->timeout);
    dict_set(out, "episode_return", log->episode_return);
    dict_set(out, "episode_length", log->episode_length);
    dict_set(out, "ema_dist", log->ema_dist);
    dict_set(out, "ema_vel", log->ema_vel);
    dict_set(out, "ema_omega", log->ema_omega);
}
