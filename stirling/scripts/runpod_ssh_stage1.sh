#!/usr/bin/env bash
# Drive a Stage 1 training run on an already-provisioned RunPod pod over proxy
# SSH. This keeps local interaction to one command after the pod exists.
#
# Example:
#   bash stirling/scripts/runpod_ssh_stage1.sh ixeoab1aw1v46m-6441200e
#
# Fast timing run:
#   STAGE1_MODE=fast bash stirling/scripts/runpod_ssh_stage1.sh ixeoab1aw1v46m-6441200e
set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <runpod-host-id> [repo-url] [branch]" >&2
    echo "Example: $0 ixeoab1aw1v46m-6441200e https://github.com/adamcroft330/adSwarm.git stirling-drone" >&2
    exit 2
fi

POD_HOST_ID="$1"
REPO_URL="${2:-https://github.com/adamcroft330/adSwarm.git}"
BRANCH="${3:-stirling-drone}"
SSH_KEY="${RUNPOD_SSH_KEY:-$HOME/.runpod/stage1_key}"
REMOTE_DIR="${RUNPOD_REMOTE_DIR:-/root/adSwarm}"
MODE="${STAGE1_MODE:-baseline}"

ssh -tt \
    -i "$SSH_KEY" \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    "${POD_HOST_ID}@ssh.runpod.io" <<EOF
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
chmod +x stirling/scripts/runpod_stage1_once.sh
STAGE1_MODE="$MODE" STAGE1_DETACH=1 bash stirling/scripts/runpod_stage1_once.sh
echo "Use a later SSH check to monitor: pgrep -af 'runpod_stage1_once|puffer train|build.sh' || true; tail -220 /root/stage1_${MODE}_*.log"
exit
EOF
