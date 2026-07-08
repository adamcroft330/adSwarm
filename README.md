# Stirling Drone Puffer

Stirling Drone Puffer is a working reinforcement-learning sandbox for the
Stirling autonomous drone-swarm project. It is based on PufferLib 4.0 and its
Ocean environments, with the current project work focused on reproducing and
extending the native `drone` environment before moving toward multi-drone
formation racing.

The repository contains:

- A Python package, `pufferlib`, with the PuffeRL trainer and command-line
  interface.
- Native C/CUDA training backends and environment bindings under `src/` and
  `ocean/`.
- The Ocean environment collection, including `drone`, `craftax`, `breakout`,
  `nmmo3`, `drive`, `impulse_wars`, and many smaller benchmark games.
- Stirling-specific planning documents and RunPod workflow scripts under
  `stirling/`.
- Tests and parity checks for kernels, encoders, sweep logic, and selected
  environments.

## Project Status

The current Stirling workflow is Stage 1: reproduce the unmodified PufferLib
Ocean `drone` HOVER baseline and record reference metrics. This provides the
regression target for later changes to platform constants, action interfaces,
observations, rewards, multi-agent formation behavior, and sim-to-real work.

Project planning lives in:

- `stirling/docs/drone_project_docs_v0_5.md`
- `stirling/docs/rl_pipelines_doc_v0_2.md`
- `stirling/docs/requirements_doc_v0_3.md`
- `stirling/docs/stage1_handoff.md`

## Requirements

For the native CUDA training path:

- Python 3.10+
- PyTorch with CUDA support
- CUDA toolkit with `nvcc`
- cuDNN and NCCL, either system-installed or available through the NVIDIA
  Python wheels used by your PyTorch install
- `clang`, OpenMP, and optionally `ccache`

For local standalone rendering builds, `build.sh` downloads the matching Raylib
release automatically.

## Training Options

You can run Stage 1 training either locally or on RunPod:

- Use the quick start path if you are setting up and running training on your
  own local machine.
- Use the RunPod Stage 1 workflow if you want to launch the same training run
  on a fresh RunPod CUDA development pod.

## Quick Start: Local Training

Create an environment, install the package, and build one Ocean environment:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip wheel
python -m pip install -e .

bash build.sh drone
python -c "import pufferlib, pufferlib._C; print('import OK')"
```

Train the drone baseline:

```bash
puffer train drone
```

Or use the Stirling Stage 1 wrapper:

```bash
bash stirling/scripts/train_stage1.sh
```

Checkpoints are written under `checkpoints/drone/<run_id>/`. Logs are written
under `logs/drone/`.

## RunPod Stage 1 Workflow

For repeatable full HOVER training on RunPod, use the single-command launcher:

```bash
RUNPOD_API_KEY_FILE=~/.runpod_key bash stirling/scripts/train_hover_runpod.sh
```

By default it creates a pod, waits for the run to finish, copies the log and
newest checkpoint under `stirling/artifacts/stage1/<run-tag>/`, and stops the
pod.

Common overrides:

```bash
STAGE1_TAG=my-hover-model \
STAGE1_TOTAL_TIMESTEPS=40000000 \
RUNPOD_API_KEY_FILE=~/.runpod_key \
bash stirling/scripts/train_hover_runpod.sh
```

For manual use on a fresh RunPod PyTorch CUDA development image:

```bash
git clone https://github.com/adamcroft330/adSwarm stirling-drone-puffer
cd stirling-drone-puffer
bash stirling/scripts/runpod_setup.sh
bash stirling/scripts/train_stage1.sh
```

Or launch the create/wait/clone/setup/build/train sequence from your local
machine in one command:

```bash
RUNPOD_API_KEY_FILE=~/.runpod_key STAGE1_MODE=train \
  RUNPOD_WAIT=1 RUNPOD_COPY_ARTIFACTS=1 RUNPOD_STOP_ON_DONE=1 \
  bash stirling/scripts/runpod_stage1_local.sh
```

For a shorter smoke/timing run that keeps the HOVER task and native CUDA path:

```bash
RUNPOD_POD_ID=<pod-id> STAGE1_MODE=fast STAGE1_SKIP_SETUP=1 \
  RUNPOD_API_KEY_FILE=~/.runpod_key bash stirling/scripts/runpod_stage1_local.sh
```

To enable Weights & Biases logging:

```bash
export WANDB_API_KEY=<your-key>
bash stirling/scripts/train_stage1.sh
```

The setup script installs native build dependencies, installs this package in
editable mode, builds the `drone` CUDA backend, and verifies that
`pufferlib._C` imports.

## Building Environments

`build.sh` compiles one environment at a time into `pufferlib/_C...so`. The
trainer checks that the compiled backend matches the environment name.

Common builds:

```bash
bash build.sh drone             # CUDA training backend
bash build.sh drone --float     # float32 precision
bash build.sh drone --cpu       # CPU fallback
bash build.sh drone --debug     # debug build
bash build.sh drone --local     # local standalone executable
bash build.sh drone --fast      # optimized standalone executable
bash build.sh drone --web       # Emscripten web build
bash build.sh all               # build all Ocean envs, default and float
```

If you switch from `drone` to another environment, rebuild first:

```bash
bash build.sh breakout
puffer train breakout
```

## Useful Commands

Run the main tests:

```bash
pytest tests
```

Run a quick import/performance smoke test:

```bash
python tests/test_import_performance.py
```

Render or experiment with examples:

```bash
python examples/render.py
python examples/pufferl.py
```

Run a hyperparameter sweep:

```bash
puffer sweep drone
```

## Repository Layout

```text
config/      Environment and training configuration files
examples/    Small Python examples for trainer, vectorization, and wrappers
ocean/       C environments and Python bindings used by PufferLib Ocean
pufferlib/   Python package and CLI entry point
resources/   Weights, textures, maps, sprites, and render assets
src/         Shared CUDA/C++ trainer, model, tensor, and binding code
stirling/    Drone-swarm project docs, scripts, and Stage 1 artifacts
tests/       Unit, parity, convergence, and kernel tests
vendor/      Vendored C headers and small support libraries
```

## Configuration

Training defaults are loaded from `config/default.ini` and then overridden by
the selected environment config, such as `config/drone.ini`. Most values can be
overridden from the CLI with dotted arguments:

```bash
puffer train drone \
  --train.total-timesteps 10000000 \
  --train.learning-rate 0.003 \
  --env.num-drones 64
```

The current `drone` config sets the HOVER task, 2,048 total agents, 64 drones
per environment instance, one GPU, and a 40M-step Stage 1 run.

## Notes for Development

- `pufferlib._C` is generated by `build.sh`; rebuild after changing C/CUDA
  environment code or switching environment targets.
- The native backend is intentionally specialized to the selected environment
  for speed.
- RunPod pod disks are ephemeral. Copy checkpoints and logs you care about
  before terminating the pod.
- The Stirling documents are part of the working design record. Update them
  when project decisions change, especially around the drone action interface,
  formation task, and validation criteria.

## Upstream

This project is based on PufferLib, a fast reinforcement-learning library and
environment suite by Puffer AI.

- Documentation: https://puffer.ai
- Upstream project: https://github.com/pufferai/pufferlib
- License: MIT, see `LICENSE`
