#!/usr/bin/env bash
# Stage 1 — baseline replication: train the UNMODIFIED drone HOVER task to
# convergence and capture reference metrics. (RL pipeline doc v0.2 §3, Stage 1.)
#
# Nothing about the env or config is changed here — this is the regression
# baseline that later stages are measured against. config/drone.ini already
# sets task=1 (HOVER), total_timesteps=40M, gpus=1.
#
# Usage (on the RunPod pod, after runpod_setup.sh):
#   bash stirling/scripts/train_stage1.sh
#
# Resume from a checkpoint:
#   bash stirling/scripts/train_stage1.sh --load-model-path checkpoints/drone/<run>/<step>.bin
set -euo pipefail
cd "$(dirname "$0")/../.."   # repo root

WANDB_ARGS=()
if [ -n "${WANDB_API_KEY:-}" ]; then
    WANDB_ARGS=(--wandb --wandb-project stirling-drone --wandb-group stage1-baseline)
fi

# checkpoint_interval is in epochs (config default 200); checkpoints land in
# checkpoints/drone/<run_id>/<step>.bin
puffer train drone \
    --tag stage1-hover-baseline \
    "${WANDB_ARGS[@]}" \
    "$@"

echo
echo "== training finished =="
echo "Checkpoints: checkpoints/drone/<run_id>/"
echo "Record the final dashboard frame (hover_score, hover_ema, ema_dist,"
echo "ema_vel, ema_omega, episode_return, episode_length, total steps,"
echo "wall-clock, steady-state SPS) and report them back to capture the"
echo "Stage 1 baseline. Copy the latest checkpoint off the pod before"
echo "terminating it (pod disk is ephemeral)."
