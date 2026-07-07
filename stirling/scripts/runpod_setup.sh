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
if ! python - <<'PY'
import torch
if not torch.cuda.is_available():
    raise SystemExit(1)
print(f"torch CUDA OK: {torch.cuda.get_device_name(0)}")
PY
then
    TORCH_FALLBACK_INDEX="${RUNPOD_TORCH_FALLBACK_INDEX:-https://download.pytorch.org/whl/cu128}"
    echo "torch CUDA preflight failed; reinstalling torch 2.9.1 from ${TORCH_FALLBACK_INDEX}"
    python -m pip install --force-reinstall torch==2.9.1 --index-url "$TORCH_FALLBACK_INDEX"
    python - <<'PY'
import torch
if not torch.cuda.is_available():
    raise SystemExit(
        "ERROR: torch.cuda.is_available() is false after torch fallback. "
        "Do not build/train on this pod; recreate it with a working GPU/driver."
    )
print(f"torch CUDA OK: {torch.__version__} {torch.cuda.get_device_name(0)}")
PY
fi

if ! ldconfig -p 2>/dev/null | grep -q 'libnvidia-ml.so '; then
    NVML_SO="$(ldconfig -p 2>/dev/null | awk '/libnvidia-ml.so.1/{print $NF; exit}')"
    if [ -n "$NVML_SO" ]; then
        ln -sf "$NVML_SO" /usr/lib/x86_64-linux-gnu/libnvidia-ml.so
        echo "linked libnvidia-ml.so -> $NVML_SO"
    fi
fi

# --- 2. Install native build dependencies -----------------------------------
# RunPod containers are ephemeral. The PyTorch image has CUDA + torch, but not
# the C/CUDA build helpers used by build.sh drone.
if command -v apt-get >/dev/null 2>&1; then
    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y clang libomp-dev ccache
else
    for tool in clang ccache; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            echo "ERROR: $tool not found and apt-get is unavailable."
            exit 1
        fi
    done
fi

# --- 3. Install PufferLib (editable) ----------------------------------------
# RunPod PyTorch images already ship torch + CUDA libs. Preserve that exact
# Torch build; the host driver/container CUDA pairing is fragile across pods.
python -m pip install --upgrade pip wheel
python -m pip install numpy rich rich_argparse gpytorch scikit-learn wandb pybind11
python -m pip install -e . --no-deps

# --- 4. Build the drone env (native CUDA backend) ---------------------------
bash build.sh drone
python -c "import pufferlib, pufferlib._C; print('pufferlib + _C import OK')"

# --- 5. wandb (optional) ----------------------------------------------------
if [ -n "${WANDB_API_KEY:-}" ]; then
    wandb login "$WANDB_API_KEY"
    echo "wandb: logged in"
else
    echo "wandb: WANDB_API_KEY not set — training will run without wandb."
    echo "       (re-run with the key exported, or pass without --wandb)"
fi

echo
echo "== setup complete. Next: bash stirling/scripts/train_stage1.sh =="
