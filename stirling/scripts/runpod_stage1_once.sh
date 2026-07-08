#!/usr/bin/env bash
# Run the Stage 1 drone HOVER workflow from inside a RunPod container.
#
# Baseline replication:
#   bash stirling/scripts/runpod_stage1_once.sh
#
# Full repeatable training run for new models:
#   STAGE1_MODE=train STAGE1_TAG=my-model bash stirling/scripts/runpod_stage1_once.sh
#
# Short optimized timing run:
#   STAGE1_MODE=fast bash stirling/scripts/runpod_stage1_once.sh
#
# Detached:
#   STAGE1_DETACH=1 bash stirling/scripts/runpod_stage1_once.sh
set -euo pipefail

if [ -n "${STAGE1_REPO_DIR:-}" ]; then
    cd "$STAGE1_REPO_DIR"
else
    cd "$(dirname "$0")/../.."
fi

MODE="${STAGE1_MODE:-baseline}"
LOG_DIR="${STAGE1_LOG_DIR:-/root}"
RUN_NAME="${STAGE1_RUN_NAME:-stage1_${MODE}_$(date -u +%Y%m%dT%H%M%SZ)}"
LOG_PATH="${STAGE1_LOG_PATH:-${LOG_DIR}/${RUN_NAME}.log}"

case "$MODE" in
    baseline)
        TRAIN_ARGS=(--tag stage1-hover-baseline)
        ;;
    train)
        TRAIN_ARGS=(--tag "${STAGE1_TAG:-stage1-hover-train}")
        if [ -n "${STAGE1_TOTAL_TIMESTEPS:-}" ]; then
            TRAIN_ARGS+=(--train.total-timesteps "$STAGE1_TOTAL_TIMESTEPS")
        fi
        if [ -n "${STAGE1_TOTAL_AGENTS:-}" ]; then
            TRAIN_ARGS+=(--vec.total-agents "$STAGE1_TOTAL_AGENTS")
        fi
        if [ -n "${STAGE1_MINIBATCH_SIZE:-}" ]; then
            TRAIN_ARGS+=(--train.minibatch-size "$STAGE1_MINIBATCH_SIZE")
        fi
        if [ -n "${STAGE1_EXTRA_ARGS:-}" ]; then
            # Shell-like quoting is intentionally not supported here. Keep
            # extra args simple: STAGE1_EXTRA_ARGS='--foo 1 --bar baz'.
            read -r -a EXTRA_ARGS <<< "$STAGE1_EXTRA_ARGS"
            TRAIN_ARGS+=("${EXTRA_ARGS[@]}")
        fi
        ;;
    fast)
        # Keeps the same HOVER task and CUDA backend, but shortens the run for
        # quick iteration/timing after the baseline has already been replicated.
        TRAIN_ARGS=(
            --tag stage1-hover-fast
            --train.total-timesteps "${STAGE1_FAST_TIMESTEPS:-10000000}"
            --vec.total-agents "${STAGE1_FAST_TOTAL_AGENTS:-4096}"
            --train.minibatch-size "${STAGE1_FAST_MINIBATCH_SIZE:-16384}"
        )
        ;;
    *)
        echo "ERROR: STAGE1_MODE must be 'baseline', 'train', or 'fast', got '$MODE'" >&2
        exit 2
        ;;
esac

run_stage1() {
    printf "stage1 mode: %s\n" "$MODE"
    printf "repo: %s\n" "$(pwd)"
    printf "commit: %s\n" "$(git rev-parse --short HEAD)"
    printf "started: "
    date -u

    if [ "${STAGE1_SKIP_SETUP:-0}" = "1" ]; then
        echo "setup: skipped (STAGE1_SKIP_SETUP=1)"
        python -c "import pufferlib, pufferlib._C; print('pufferlib + _C import OK')"
    else
        bash stirling/scripts/runpod_setup.sh
    fi

    printf "training args:"
    printf " %q" "${TRAIN_ARGS[@]}"
    printf "\n"
    bash stirling/scripts/train_stage1.sh "${TRAIN_ARGS[@]}"

    printf "finished: "
    date -u
    find checkpoints/drone -maxdepth 3 -type f -name '*.bin' -printf '%p %s bytes\n' | sort || true
}

if [ "${STAGE1_DETACH:-0}" = "1" ]; then
    mkdir -p "$LOG_DIR"
    STAGE1_DETACH=0 \
    STAGE1_MODE="$MODE" \
    STAGE1_LOG_PATH="$LOG_PATH" \
    STAGE1_RUN_NAME="$RUN_NAME" \
    STAGE1_SKIP_SETUP="${STAGE1_SKIP_SETUP:-0}" \
    STAGE1_TAG="${STAGE1_TAG:-}" \
    STAGE1_TOTAL_TIMESTEPS="${STAGE1_TOTAL_TIMESTEPS:-}" \
    STAGE1_TOTAL_AGENTS="${STAGE1_TOTAL_AGENTS:-}" \
    STAGE1_MINIBATCH_SIZE="${STAGE1_MINIBATCH_SIZE:-}" \
    STAGE1_EXTRA_ARGS="${STAGE1_EXTRA_ARGS:-}" \
    STAGE1_FAST_TIMESTEPS="${STAGE1_FAST_TIMESTEPS:-10000000}" \
    STAGE1_FAST_TOTAL_AGENTS="${STAGE1_FAST_TOTAL_AGENTS:-4096}" \
    STAGE1_FAST_MINIBATCH_SIZE="${STAGE1_FAST_MINIBATCH_SIZE:-16384}" \
        nohup setsid bash "$0" > "$LOG_PATH" 2>&1 &
    echo "STAGE1_PID:$!"
    echo "STAGE1_LOG:$LOG_PATH"
    echo "Monitor with: tail -f $LOG_PATH"
else
    run_stage1 2>&1 | tee "$LOG_PATH"
fi
