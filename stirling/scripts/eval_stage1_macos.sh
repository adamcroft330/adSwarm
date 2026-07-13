#!/usr/bin/env bash
# Evaluate a Stage 1 drone PyTorch checkpoint on macOS using the CPU backend.
set -euo pipefail

cd "$(dirname "$0")/../.."   # repo root

if [ $# -ne 1 ]; then
    echo "Usage: bash stirling/scripts/eval_stage1_macos.sh <checkpoint.bin>"
    echo
    echo "Expected checkpoint example:"
    echo "  stirling/artifacts/stage1/0000000039976960.bin"
    echo
    echo "The Stage 1 checkpoint is not committed to this repo. Train one with"
    echo "'modal run stirling/modal/train_drone.py' (produces a .pt), or convert"
    echo "a native .bin via stirling/scripts/convert_native_checkpoint.py."
    echo
    echo "For a no-checkpoint smoke test, run:"
    echo "  puffer eval drone --slowly"
    exit 2
fi

CHECKPOINT="$1"
if [ ! -f "$CHECKPOINT" ]; then
    echo "ERROR: checkpoint not found: $CHECKPOINT"
    exit 1
fi

if command -v puffer >/dev/null 2>&1; then
    PUFFER=(puffer)
elif [ -x ".venv-macos-eval/bin/puffer" ]; then
    PUFFER=(.venv-macos-eval/bin/puffer)
else
    echo "ERROR: puffer command not found."
    echo "Run: bash stirling/scripts/macos_eval_setup.sh"
    exit 1
fi

# --slowly selects the PyTorch policy path, which can load torch checkpoints on
# CPU. The native CPU backend reports gpu=0 and provides the Raylib renderer.
"${PUFFER[@]}" eval drone --slowly --load-model-path "$CHECKPOINT"
