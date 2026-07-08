#!/usr/bin/env bash
# One-command repeatable RunPod training entrypoint for new HOVER models.
#
# This creates a pod, waits for training to finish, copies the log and newest
# checkpoint locally, then stops the pod.
set -euo pipefail

cd "$(dirname "$0")/../.."

export STAGE1_MODE="${STAGE1_MODE:-train}"
export STAGE1_TAG="${STAGE1_TAG:-hover-train-$(date -u +%Y%m%dT%H%M%SZ)}"
export RUNPOD_WAIT="${RUNPOD_WAIT:-1}"
export RUNPOD_COPY_ARTIFACTS="${RUNPOD_COPY_ARTIFACTS:-1}"
export RUNPOD_STOP_ON_DONE="${RUNPOD_STOP_ON_DONE:-1}"
export RUNPOD_API_KEY_FILE="${RUNPOD_API_KEY_FILE:-$HOME/.runpod_key}"
export RUNPOD_ARTIFACT_DIR="${RUNPOD_ARTIFACT_DIR:-stirling/artifacts/stage1/${STAGE1_TAG}}"

bash stirling/scripts/runpod_stage1_local.sh
