#!/usr/bin/env bash
# Stage 1 — RunPod provisioning for the PufferLib drone HOVER baseline.
#
# Run this once on a fresh RunPod pod (RunPod PyTorch 2.x CUDA template),
# from the root of the cloned stirling-drone-puffer repo:
#
#   git clone <your-fork-url> stirling-drone-puffer
#   cd stirling-drone-puffer && git checkout stirling-drone
#   bash stirling/scripts/runpod_setup.sh
#
# Optional: export WANDB_API_KEY=<key> before running to enable wandb logging.
set -euo pipefail

cd "$(dirname "$0")/../.."   # repo root
echo "== repo: $(pwd)  branch: $(git rev-parse --abbrev-ref HEAD) =="

# --- 1. Sanity-check the CUDA toolchain -------------------------------------
if ! command -v nvcc >/dev/null 2>&1; then
    echo "ERROR: nvcc not found. Use a RunPod template with the full CUDA"
    echo "toolkit (a *-devel image), not a runtime-only one."
    exit 1
fi
nvcc --version | tail -1
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader || true

# --- 2. Install PufferLib (editable) ----------------------------------------
# RunPod PyTorch images already ship torch + CUDA libs; this adds the rest.
python -m pip install --upgrade pip wheel
python -m pip install -e .

# --- 3. Build the drone env (native CUDA backend) ---------------------------
bash build.sh drone
python -c "import pufferlib, pufferlib._C; print('pufferlib + _C import OK')"

# --- 4. wandb (optional) ----------------------------------------------------
if [ -n "${WANDB_API_KEY:-}" ]; then
    wandb login "$WANDB_API_KEY"
    echo "wandb: logged in"
else
    echo "wandb: WANDB_API_KEY not set — training will run without wandb."
    echo "       (re-run with the key exported, or pass without --wandb)"
fi

echo
echo "== setup complete. Next: bash stirling/scripts/train_stage1.sh =="
