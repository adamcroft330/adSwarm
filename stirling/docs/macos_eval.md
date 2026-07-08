# macOS Drone Visual Eval

This path is for local visual evaluation and development loops on macOS. It is
not intended for useful-speed training. Serious Stage 1 training remains on the
RunPod/CUDA workflow in `stirling/docs/stage1_handoff.md`.

## Quick Usage

```bash
bash stirling/scripts/macos_eval_setup.sh
bash stirling/scripts/eval_pretrained_drone_macos.sh
```

To evaluate a copied Stage 1 PyTorch checkpoint instead:

```bash
bash stirling/scripts/eval_stage1_macos.sh stirling/artifacts/stage1/0000000039976960.bin
```

## Setup

Prerequisites:

- macOS with Homebrew
- Python 3
- Homebrew packages: `libomp`, `raylib`, `ccache`

Run the setup script from the repo root:

```bash
bash stirling/scripts/macos_eval_setup.sh
```

The script installs missing Homebrew packages, creates `.venv-macos-eval` unless
you already have an active virtualenv, installs the repo in editable mode,
builds the CPU backend with the Homebrew `libomp` include and library paths, and
verifies:

```bash
python3 -c "import pufferlib, pufferlib._C as C; print(C.env_name, C.gpu)"
```

Expected output includes:

```text
drone 0
```

If the setup script created the venv, activate it before running direct `puffer`
commands:

```bash
source .venv-macos-eval/bin/activate
```

## Stage 1 Checkpoint Eval

The successful Stage 1 RunPod checkpoint was produced at:

```text
/root/adSwarm/checkpoints/drone/1778957678487/0000000039976960.bin
```

That checkpoint is not preserved in this repo. Copy a compatible `.bin`
checkpoint locally, for example:

```text
stirling/artifacts/stage1/0000000039976960.bin
```

Then run:

```bash
bash stirling/scripts/eval_stage1_macos.sh stirling/artifacts/stage1/0000000039976960.bin
```

The wrapper runs:

```bash
puffer eval drone --slowly --load-model-path "$checkpoint"
```

`--slowly` is required for checkpoint eval on macOS because it selects the
PyTorch policy path and maps the checkpoint onto CPU. The non-`--slowly` CUDA
training backend is not available on macOS.

## No-Checkpoint Smoke Tests

If no Stage 1 checkpoint is available, the local renderer can still be smoke
tested.

Random PyTorch-policy render path:

```bash
puffer eval drone --slowly
```

Native packaged-weights renderer path:

```bash
bash stirling/scripts/eval_pretrained_drone_macos.sh
```

The native path uses the packaged `resources/drone/drone_weights.bin` through
the standalone C drone executable. That file is not the same artifact type as a
PyTorch checkpoint passed to `--load-model-path`.

## Limitations

- macOS support here is eval/dev-loop only.
- Local CPU training is not a Stage 1 convergence target.
- CUDA, cuDNN, NCCL, and NVIDIA libraries are intentionally not required for
  `bash build.sh drone --cpu`.
- Checkpoint compatibility still depends on the policy architecture and config
  matching the checkpoint that was produced on RunPod.
