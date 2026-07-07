#!/usr/bin/env bash
# Local one-command RunPod launcher for Stirling Stage 1.
#
# Creates a secure RunPod pod unless RUNPOD_POD_ID is set, waits for SSH,
# clones/updates the repo, and starts pod-side training detached.
#
# Baseline:
#   RUNPOD_API_KEY_FILE=~/.runpod_key bash stirling/scripts/runpod_stage1_local.sh
#
# Full repeatable training run that waits, copies artifacts, and stops:
#   RUNPOD_API_KEY_FILE=~/.runpod_key STAGE1_MODE=train STAGE1_TAG=my-model \
#     RUNPOD_WAIT=1 RUNPOD_COPY_ARTIFACTS=1 RUNPOD_STOP_ON_DONE=1 \
#     bash stirling/scripts/runpod_stage1_local.sh
set -euo pipefail

cd "$(dirname "$0")/../.."

API_KEY_FILE="${RUNPOD_API_KEY_FILE:-$HOME/.runpod_key}"
SSH_KEY="${RUNPOD_SSH_KEY:-$HOME/.runpod/stage1_key}"
SSH_PUBLIC_KEY_FILE="${RUNPOD_SSH_PUBLIC_KEY_FILE:-${SSH_KEY}.pub}"
POD_ID="${RUNPOD_POD_ID:-}"
MODE="${STAGE1_MODE:-baseline}"
SKIP_SETUP="${STAGE1_SKIP_SETUP:-0}"
RUN_NAME="${STAGE1_RUN_NAME:-stage1_${MODE}_$(date -u +%Y%m%dT%H%M%SZ)}"
REMOTE_LOG_DIR="${STAGE1_LOG_DIR:-/root}"
REMOTE_LOG_PATH="${STAGE1_LOG_PATH:-${REMOTE_LOG_DIR}/${RUN_NAME}.log}"
WAIT_FOR_DONE="${RUNPOD_WAIT:-0}"
COPY_ARTIFACTS="${RUNPOD_COPY_ARTIFACTS:-0}"
STOP_ON_DONE="${RUNPOD_STOP_ON_DONE:-0}"
ARTIFACT_DIR="${RUNPOD_ARTIFACT_DIR:-stirling/artifacts/stage1/${RUN_NAME}}"
REPO_URL="${RUNPOD_REPO_URL:-https://github.com/adamcroft330/adSwarm.git}"
BRANCH="${RUNPOD_BRANCH:-stirling-drone}"
REMOTE_DIR="${RUNPOD_REMOTE_DIR:-/root/adSwarm}"
IMAGE_NAME="${RUNPOD_IMAGE_NAME:-runpod/pytorch:2.8.0-py3.11-cuda12.8.1-cudnn-devel-ubuntu22.04}"
POD_NAME="${RUNPOD_POD_NAME:-stirling-stage1-hover}"
POLL_SECONDS="${RUNPOD_POLL_SECONDS:-15}"
TIMEOUT_SECONDS="${RUNPOD_TIMEOUT_SECONDS:-1800}"
PYTHON_BIN="${PYTHON:-}"

GPU_TYPES_JSON="${RUNPOD_GPU_TYPES_JSON:-[\"NVIDIA GeForce RTX 4090\",\"NVIDIA RTX A6000\",\"NVIDIA RTX A5000\",\"NVIDIA L40\",\"NVIDIA A100 80GB PCIe\"]}"

api_key() {
    tr -d '\n' < "$API_KEY_FILE"
}

runpod_rest() {
    curl -fsS -H "Authorization: Bearer $(api_key)" "$@"
}

runpod_graphql() {
    curl -fsS https://api.runpod.io/graphql \
        -H "Authorization: Bearer $(api_key)" \
        -H "Content-Type: application/json" \
        -d "$1"
}

json_string() {
    "$PYTHON_BIN" -c 'import json, sys; print(json.dumps(sys.stdin.read().strip()))'
}

require_file() {
    if [ ! -f "$1" ]; then
        echo "ERROR: missing file: $1" >&2
        exit 2
    fi
}

require_file "$API_KEY_FILE"
require_file "$SSH_KEY"
require_file "$SSH_PUBLIC_KEY_FILE"

if [ -z "$PYTHON_BIN" ]; then
    if command -v python3 >/dev/null 2>&1; then
        PYTHON_BIN=python3
    elif command -v python >/dev/null 2>&1; then
        PYTHON_BIN=python
    else
        echo "ERROR: python3 or python is required locally for JSON parsing." >&2
        exit 2
    fi
fi

if [ -z "$POD_ID" ]; then
    PUBLIC_KEY_JSON="$(json_string < "$SSH_PUBLIC_KEY_FILE")"
    CREATE_BODY="$(cat <<JSON
{
  "cloudType": "SECURE",
  "computeType": "GPU",
  "containerDiskInGb": 30,
  "volumeInGb": 0,
  "volumeMountPath": "/workspace",
  "imageName": "$IMAGE_NAME",
  "gpuCount": 1,
  "gpuTypeIds": $GPU_TYPES_JSON,
  "gpuTypePriority": "availability",
  "ports": ["8888/http", "22/tcp"],
  "supportPublicIp": true,
  "name": "$POD_NAME",
  "env": {
    "PUBLIC_KEY": $PUBLIC_KEY_JSON,
    "NVIDIA_VISIBLE_DEVICES": "all",
    "NVIDIA_DRIVER_CAPABILITIES": "all"
  }
}
JSON
)"
    echo "Creating RunPod pod: $POD_NAME"
    CREATE_RESPONSE="$(runpod_rest -X POST https://rest.runpod.io/v1/pods \
        -H "Content-Type: application/json" \
        --data "$CREATE_BODY")"
    POD_ID="$(printf '%s' "$CREATE_RESPONSE" | "$PYTHON_BIN" -c 'import json, sys; print(json.load(sys.stdin)["id"])')"
    echo "POD_ID:$POD_ID"
else
    echo "Using existing RunPod pod: $POD_ID"
fi

deadline=$((SECONDS + TIMEOUT_SECONDS))
SSH_USER=""
SSH_HOST=""
SSH_PORT=""
echo "Waiting for RunPod runtime/SSH..."
while [ "$SECONDS" -lt "$deadline" ]; do
    STATUS_JSON="$(runpod_graphql "{\"query\":\"query { pod(input: {podId: \\\"$POD_ID\\\"}) { id desiredStatus machineId runtime { ports { ip isIpPublic privatePort publicPort type } uptimeInSeconds } } }\"}")"
    desired="$(printf '%s' "$STATUS_JSON" | "$PYTHON_BIN" -c 'import json, sys; print(json.load(sys.stdin)["data"]["pod"]["desiredStatus"])')"
    machine_id="$(printf '%s' "$STATUS_JSON" | "$PYTHON_BIN" -c 'import json, sys; print((json.load(sys.stdin)["data"]["pod"].get("machineId") or ""))')"
    runtime="$(printf '%s' "$STATUS_JSON" | "$PYTHON_BIN" -c 'import json, sys; print(json.load(sys.stdin)["data"]["pod"]["runtime"] is not None)')"
    echo "status=$desired runtime=$runtime machine=$machine_id"

    public_ssh="$(printf '%s' "$STATUS_JSON" | "$PYTHON_BIN" -c '
import json, sys
pod = json.load(sys.stdin)["data"]["pod"]
runtime = pod.get("runtime") or {}
for port in runtime.get("ports") or []:
    if port.get("privatePort") == 22 and port.get("type") == "tcp" and port.get("isIpPublic"):
        print("{}:{}".format(port["ip"], port["publicPort"]))
        break
')"
    if [ -n "$public_ssh" ]; then
        SSH_HOST="${public_ssh%:*}"
        SSH_PORT="${public_ssh##*:}"
        if printf 'echo SSH_OK\nexit\n' | ssh -tt \
            -p "$SSH_PORT" \
            -o ConnectTimeout=8 \
            -o ServerAliveInterval=5 \
            -o ServerAliveCountMax=1 \
            -o BatchMode=yes \
            -i "$SSH_KEY" \
            -o StrictHostKeyChecking=no \
            -o UserKnownHostsFile=/dev/null \
            "root@$SSH_HOST" 2>/dev/null | grep -q SSH_OK; then
            SSH_USER="root"
            break
        fi
    fi

    candidates=("$POD_ID")
    if [ -n "$machine_id" ]; then
        candidates+=("${POD_ID}-${machine_id}")
    fi

    for candidate in "${candidates[@]}"; do
        if printf 'echo SSH_OK\nexit\n' | ssh -tt \
            -o ConnectTimeout=8 \
            -o ServerAliveInterval=5 \
            -o ServerAliveCountMax=1 \
            -o BatchMode=yes \
            -i "$SSH_KEY" \
            -o StrictHostKeyChecking=no \
            -o UserKnownHostsFile=/dev/null \
            "${candidate}@ssh.runpod.io" 2>/dev/null | grep -q SSH_OK; then
            SSH_USER="$candidate"
            break 2
        fi
    done

    sleep "$POLL_SECONDS"
done

if [ -z "$SSH_USER" ]; then
    echo "ERROR: timed out waiting for RunPod SSH. Pod is still: $POD_ID" >&2
    exit 1
fi

echo "SSH_USER:$SSH_USER"
echo "Launching Stage 1 mode=$MODE"

if [ -n "$SSH_HOST" ] && [ -n "$SSH_PORT" ]; then
    scp -P "$SSH_PORT" \
        -i "$SSH_KEY" \
        -o StrictHostKeyChecking=no \
        -o UserKnownHostsFile=/dev/null \
        stirling/scripts/runpod_stage1_once.sh \
        stirling/scripts/runpod_setup.sh \
        "root@$SSH_HOST:/root/"
    ssh -tt \
        -p "$SSH_PORT" \
        -i "$SSH_KEY" \
        -o StrictHostKeyChecking=no \
        -o UserKnownHostsFile=/dev/null \
        "root@$SSH_HOST" <<EOF
set -euo pipefail
cd /root
if [ ! -d "$REMOTE_DIR/.git" ]; then
    rm -rf "$REMOTE_DIR"
    git clone --depth 1 --branch "$BRANCH" --progress "$REPO_URL" "$REMOTE_DIR"
else
    cd "$REMOTE_DIR"
    git fetch --depth 1 origin "$BRANCH"
    git checkout "$BRANCH"
    git reset --hard "origin/$BRANCH"
fi
cd "$REMOTE_DIR"
cp /root/runpod_stage1_once.sh stirling/scripts/runpod_stage1_once.sh
cp /root/runpod_setup.sh stirling/scripts/runpod_setup.sh
chmod +x stirling/scripts/runpod_stage1_once.sh
STAGE1_REPO_DIR="$REMOTE_DIR" STAGE1_MODE="$MODE" STAGE1_RUN_NAME="$RUN_NAME" STAGE1_LOG_PATH="$REMOTE_LOG_PATH" STAGE1_SKIP_SETUP="$SKIP_SETUP" STAGE1_TAG="${STAGE1_TAG:-}" STAGE1_TOTAL_TIMESTEPS="${STAGE1_TOTAL_TIMESTEPS:-}" STAGE1_TOTAL_AGENTS="${STAGE1_TOTAL_AGENTS:-}" STAGE1_MINIBATCH_SIZE="${STAGE1_MINIBATCH_SIZE:-}" STAGE1_EXTRA_ARGS="${STAGE1_EXTRA_ARGS:-}" STAGE1_DETACH=1 bash /root/runpod_stage1_once.sh
exit
EOF
else
    bash stirling/scripts/runpod_ssh_stage1.sh "$SSH_USER" "$REPO_URL" "$BRANCH"
fi

echo
echo "Pod: $POD_ID"
echo "SSH user: $SSH_USER"
echo "Monitor:"
if [ -n "$SSH_HOST" ] && [ -n "$SSH_PORT" ]; then
    echo "  printf 'pgrep -af \"runpod_stage1_once|puffer train|build.sh\" || true; tail -220 /root/stage1_${MODE}_*.log; exit\n' | ssh -tt -p \"$SSH_PORT\" -i \"$SSH_KEY\" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \"root@$SSH_HOST\""
else
    echo "  printf 'pgrep -af \"runpod_stage1_once|puffer train|build.sh\" || true; tail -220 /root/stage1_${MODE}_*.log; exit\n' | ssh -tt -i \"$SSH_KEY\" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \"$SSH_USER@ssh.runpod.io\""
fi

if [ "$WAIT_FOR_DONE" = "1" ]; then
    if [ -z "$SSH_HOST" ] || [ -z "$SSH_PORT" ]; then
        echo "ERROR: RUNPOD_WAIT=1 currently requires direct public SSH runtime." >&2
        exit 1
    fi

    echo "Waiting for training to finish: $REMOTE_LOG_PATH"
    while true; do
        status="$(printf 'pgrep -af "runpod_stage1_once|puffer train|build.sh" || true\nif [ -f "%s" ]; then tail -40 "%s"; fi\nexit\n' "$REMOTE_LOG_PATH" "$REMOTE_LOG_PATH" | ssh -tt -p "$SSH_PORT" -i "$SSH_KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "root@$SSH_HOST" 2>&1 || true)"
        printf '%s\n' "$status" | tail -60
        if printf '%s\n' "$status" | grep -q '== training finished =='; then
            break
        fi
        if printf '%s\n' "$status" | grep -Eiq 'Traceback|ERROR:|Aborted|core dumped|Assertion `|No such file or directory|failed'; then
            echo "ERROR: training appears to have failed. Pod left running for inspection: $POD_ID" >&2
            exit 1
        fi
        sleep "${RUNPOD_MONITOR_SECONDS:-30}"
    done

    if [ "$COPY_ARTIFACTS" = "1" ]; then
        mkdir -p "$ARTIFACT_DIR"
        remote_latest="$(printf 'find "%s/checkpoints/drone" -type f -name "*.bin" -printf "%%T@ %%p\\n" 2>/dev/null | sort -n | tail -1 | cut -d" " -f2-\nexit\n' "$REMOTE_DIR" | ssh -tt -p "$SSH_PORT" -i "$SSH_KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "root@$SSH_HOST" 2>/dev/null | tr -d '\r' | tail -1)"
        scp -P "$SSH_PORT" -i "$SSH_KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            "root@$SSH_HOST:$REMOTE_LOG_PATH" "$ARTIFACT_DIR/"
        if [ -n "$remote_latest" ]; then
            scp -P "$SSH_PORT" -i "$SSH_KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
                "root@$SSH_HOST:$remote_latest" "$ARTIFACT_DIR/"
        fi
        echo "Artifacts copied to: $ARTIFACT_DIR"
    fi

    if [ "$STOP_ON_DONE" = "1" ]; then
        curl -fsS -X POST -H "Authorization: Bearer $(api_key)" "https://rest.runpod.io/v1/pods/$POD_ID/stop" >/dev/null
        echo "Stopped pod: $POD_ID"
    fi
fi
