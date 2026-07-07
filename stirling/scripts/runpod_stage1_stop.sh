#!/usr/bin/env bash
# Stop a RunPod pod by id.
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <pod-id>" >&2
    exit 2
fi

API_KEY_FILE="${RUNPOD_API_KEY_FILE:-$HOME/.runpod_key}"
tr -d '\n' < "$API_KEY_FILE" >/dev/null

curl -fsS -X POST \
    -H "Authorization: Bearer $(tr -d '\n' < "$API_KEY_FILE")" \
    "https://rest.runpod.io/v1/pods/$1/stop"
echo
