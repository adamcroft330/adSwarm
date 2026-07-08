# Stage 1 RunPod Handoff - Stirling Drone Project

Written 2026-05-16. Updated after the successful Stage 1 HOVER baseline run.

## Status

Stage 1 baseline training has been run successfully on RunPod.

- Pod id: `ixeoab1aw1v46m`
- RunPod SSH host: `ixeoab1aw1v46m-6441200e@ssh.runpod.io`
- GPU: NVIDIA GeForce RTX 5090, 32607 MiB
- Image: `runpod/pytorch:1.0.3-cu1290-torch291-ubuntu2204`
- Repo on pod: `/root/adSwarm`
- Branch/commit trained: `stirling-drone` / `7037d83`
- Training log: `/root/build_and_train.log`
- Checkpoint dir on pod: `/root/adSwarm/checkpoints/drone/1778957678487/`
- Final checkpoint on pod: `/root/adSwarm/checkpoints/drone/1778957678487/0000000039976960.bin`

The pod disk is ephemeral. Copy the checkpoint off the pod before terminating it
if it has not already been copied.

For local macOS visual evaluation after copying a checkpoint, see
`stirling/docs/macos_eval.md`. The macOS path is eval/dev-loop only; Stage 1
training remains the RunPod/CUDA workflow documented here.

## Objective

Stage 1 baseline replication trains the unmodified PufferLib 4.0 Ocean `drone`
HOVER task to convergence using the native motor-level action space. This is the
regression target for later Stirling formation-control work.

Stage 1 intentionally does not modify `config/drone.ini`:

- `task=1` (HOVER)
- `total_timesteps=40000000`
- `gpus=1`

## Final Metrics

Final values from `/root/build_and_train.log`:

| Metric | Value |
| --- | ---: |
| Steps | `40.0M` |
| SPS | `2.7M` |
| GPU | `85%` |
| VRAM | `0.7/31G` |
| score | `836.861` |
| episode_return | `55.996` |
| episode_length | `1020.091` |
| ema_dist | `0.020` |
| ema_vel | `0.045` |
| ema_omega | `0.151` |

Completion audit evidence:

- `python -c "import pufferlib, pufferlib._C; print('pufferlib + _C import OK')"`
  succeeded on the pod.
- No `puffer train`, `build_and_train`, or `train_stage1` process remained after
  the run.
- `/root/build_and_train.log` ended with `== training finished ==`.
- Checkpoints were present:
  - `0000000000065536.bin`
  - `0000000013172736.bin`
  - `0000000026279936.bin`
  - `0000000039387136.bin`
  - `0000000039976960.bin`

## 2026-05-17 Repeat Run

The baseline was rerun on RunPod and followed by an optimized short repeat.

- Pod id: `wabj62go2ch81f`
- GPU: NVIDIA GeForce RTX 4090, 24564 MiB
- Image: `runpod/pytorch:2.8.0-py3.11-cuda12.8.1-cudnn-devel-ubuntu22.04`
- Repo on pod: `/root/adSwarm`
- Branch/commit trained: `stirling-drone` / `5a88a98`
- Pod status after artifact copy: `EXITED`
- Local artifacts:
  - `stirling/artifacts/stage1/runpod_20260517/stage1_baseline_20260517T105523Z.log`
  - `stirling/artifacts/stage1/runpod_20260517/0000000039976960.bin`
  - `stirling/artifacts/stage1/runpod_20260517/stage1_fast_20260517T105842Z.log`
  - `stirling/artifacts/stage1/runpod_20260517/0000000009961472.bin`

Baseline repeat final values:

| Metric | Value |
| --- | ---: |
| Steps | `40.0M` |
| SPS | `2.4M` |
| GPU | `81%` |
| VRAM | `0.9/24G` |
| score | `839.450` |
| episode_return | `56.549` |
| episode_length | `1013.617` |
| ema_dist | `0.041` |
| ema_vel | `0.089` |
| ema_omega | `0.259` |

Optimized fast repeat:

- Mode: `STAGE1_MODE=fast STAGE1_SKIP_SETUP=1`
- CLI overrides:
  - `--train.total-timesteps 10000000`
  - `--vec.total-agents 4096`
  - `--train.minibatch-size 16384`

| Metric | Value |
| --- | ---: |
| Steps | `10.0M` |
| SPS | `4.2M` |
| GPU | `80%` |
| VRAM | `1.0/24G` |
| score | `339.059` |
| episode_return | `12.987` |
| episode_length | `1020.349` |
| ema_dist | `0.257` |
| ema_vel | `0.487` |
| ema_omega | `1.529` |

The fast repeat is a throughput/timing run, not a converged baseline.

## Exact Replication Steps

These steps reproduce the successful run on a fresh RunPod secure-cloud pod.

### 1. Provision the pod

Use a secure-cloud pod. Community-cloud pods were unreliable during bring-up.

Recommended RunPod settings:

- `cloudType`: `SECURE`
- `imageName`: `runpod/pytorch:2.8.0-py3.11-cuda12.8.1-cudnn-devel-ubuntu22.04`
- `gpuCount`: `1`
- `containerDiskInGb`: `30`
- `volumeInGb`: `0`
- `ports`: `["8888/http", "22/tcp"]`
- `supportPublicIp`: `true`
- `gpuTypeIds`: broad list such as RTX 4090, RTX A6000, RTX A5000,
  L40, A100 80GB PCIe

The first successful pod used the older
`runpod/pytorch:1.0.3-cu1290-torch291-ubuntu2204` image on an RTX 5090. Later
570-driver hosts exposed `nvidia-smi` but failed Torch CUDA initialization with
error 804 (`forward compatibility was attempted on non supported HW`) when using
that CUDA 12.9 image. Treat `torch.cuda.is_available()` as the gate, not
`nvidia-smi` alone. The local launcher now defaults to RunPod's official PyTorch
2.8 + CUDA 12.8 image and `runpod_setup.sh` preserves the image's Torch build
instead of letting editable install upgrade it.

### 2. Connect over RunPod proxy SSH

RunPod proxy SSH ignores one-shot remote commands and opens an interactive shell.
Drive commands through stdin:

```bash
printf 'hostname\nnvidia-smi --query-gpu=name,memory.total --format=csv,noheader\nwhich python\npython --version\nexit\n' \
  | ssh -tt -i ~/.runpod/stage1_key \
      -o StrictHostKeyChecking=no \
      -o UserKnownHostsFile=/dev/null \
      <POD_HOST>@ssh.runpod.io
```

`<POD_HOST>` is the pod host id from RunPod GraphQL, for example
`ixeoab1aw1v46m-6441200e`.

On the successful run the dashboard-provided `~/.ssh/id_ed25519` command did not
authenticate. The registered project key `~/.runpod/stage1_key` did.

### 3. Install missing system packages

The RunPod PyTorch image does not include all build tools required by
`build.sh drone`. `stirling/scripts/runpod_setup.sh` now installs these
automatically on Ubuntu/Debian images with `apt-get`, but the explicit packages
are:

```bash
apt-get update
apt-get install -y clang libomp-dev ccache
```

Why each package is needed:

- `clang`: `build.sh` uses `clang` for the static Ocean environment library.
- `libomp-dev`: provides `omp.h` for `src/vecenv.h`.
- `ccache`: `build.sh` invokes `ccache` for the CUDA backend compile.

### 4. Shallow-clone the training branch

A full clone stalled for several minutes in `git index-pack`. The successful run
used a shallow branch clone:

```bash
cd /root
rm -rf adSwarm
git clone --depth 1 --branch stirling-drone --progress \
  https://github.com/adamcroft330/adSwarm.git adSwarm
cd /root/adSwarm
git rev-parse --short HEAD
```

Expected commit for the recorded baseline: `7037d83`.

### 5. Install Python deps and build the backend

Normal path:

```bash
cd /root/adSwarm
bash stirling/scripts/runpod_setup.sh
```

Equivalent expanded commands:

```bash
cd /root/adSwarm
python -m pip install --upgrade pip wheel
python -m pip install -e .
bash build.sh drone
python -c "import pufferlib, pufferlib._C; print('pufferlib + _C import OK')"
```

Do not continue to training unless the `_C` import succeeds. If training starts
without `_C`, it fails immediately with:

```text
ImportError: Failed to import PufferLib C++ backend.
```

### 6. Launch training detached

Long commands must run detached because the proxy SSH pipe is fragile. The
successful run used this script:

```bash
cat > /root/build_and_train.sh <<'EOS'
#!/usr/bin/env bash
set -euo pipefail
cd /root/adSwarm
printf "build started: "
date -u
bash stirling/scripts/runpod_setup.sh
printf "train started: "
date -u
exec bash stirling/scripts/train_stage1.sh
EOS

chmod +x /root/build_and_train.sh
nohup setsid bash /root/build_and_train.sh > /root/build_and_train.log 2>&1 &
echo "BUILD_TRAIN_PID:$!"
```

The repo now also has a reusable pod-side wrapper for the same flow:

```bash
STAGE1_DETACH=1 bash stirling/scripts/runpod_stage1_once.sh
```

If the pod is already provisioned and reachable over RunPod proxy SSH, the
local one-command launcher can clone/update the repo, run setup, and start
detached training:

```bash
bash stirling/scripts/runpod_ssh_stage1.sh <POD_HOST>
```

`<POD_HOST>` is the host id before `@ssh.runpod.io`, for example
`ixeoab1aw1v46m-6441200e`.

This run was executed without W&B because proxy-SSH command echoing made secret
injection unsafe. `~/.wandb_key` was validated beforehand against W&B, but the
key was not placed on the pod.

### 6b. Optimized repeat/timing run

For training new models, use the full training wrapper rather than `fast` mode:

```bash
RUNPOD_API_KEY_FILE=~/.runpod_key bash stirling/scripts/train_hover_runpod.sh
```

The wrapper creates a pod, waits for training to finish, copies the log and
newest checkpoint into `stirling/artifacts/stage1/<run-tag>/`, and stops the pod.
Use these common overrides:

```bash
STAGE1_TAG=my-hover-model \
STAGE1_TOTAL_TIMESTEPS=40000000 \
RUNPOD_API_KEY_FILE=~/.runpod_key \
bash stirling/scripts/train_hover_runpod.sh
```

Optional training overrides:

- `STAGE1_TOTAL_TIMESTEPS`
- `STAGE1_TOTAL_AGENTS`
- `STAGE1_MINIBATCH_SIZE`
- `STAGE1_EXTRA_ARGS`

After the full baseline has been replicated, use fast mode only for a shorter
smoke/timing run when the goal is to compare setup/build/training throughput:

```bash
STAGE1_MODE=fast bash stirling/scripts/runpod_ssh_stage1.sh <POD_HOST>
```

Fast mode keeps `task=1` (HOVER) and the native CUDA backend, but passes CLI
overrides instead of changing `config/drone.ini`:

- `--train.total-timesteps ${STAGE1_FAST_TIMESTEPS:-10000000}`
- `--vec.total-agents ${STAGE1_FAST_TOTAL_AGENTS:-4096}`
- `--train.minibatch-size ${STAGE1_FAST_MINIBATCH_SIZE:-16384}`

Override those environment variables locally if a pod/GPU needs different
batch sizing.

When repeating on the same pod after a successful setup/build, skip that work:

```bash
RUNPOD_POD_ID=<POD_ID> STAGE1_MODE=fast STAGE1_SKIP_SETUP=1 \
  bash stirling/scripts/runpod_stage1_local.sh
```

### 7. Monitor progress

Reconnect periodically and inspect process state plus the log:

```bash
pgrep -af "build_and_train|build.sh|ccache|nvcc|puffer train|python" || true
tail -220 /root/build_and_train.log
```

Expected phases:

1. `Compiling static library for drone...`
2. `Compiling CUDA (native) training backend...`
3. `_C` import check prints `pufferlib + _C import OK`
4. PufferLib dashboard appears for `Env drone`
5. Run reaches `Steps 40.0M`
6. Log ends with `== training finished ==`

### 8. Verify completion

After the log says training finished:

```bash
cd /root/adSwarm
python -c "import pufferlib, pufferlib._C; print('pufferlib + _C import OK')"
pgrep -af "puffer train|build_and_train|train_stage1" || true
find /root/adSwarm/checkpoints -maxdepth 4 -type f -printf "%p %s bytes\n" | sort
tail -80 /root/build_and_train.log
```

For the successful run, `pgrep` returned no training process and checkpoints were
written under:

```text
/root/adSwarm/checkpoints/drone/1778957678487/
```

### 9. Copy the checkpoint off the pod

The final checkpoint is small, about 604 KB. If direct `scp` through the proxy is
not convenient, base64 it over SSH:

```bash
base64 /root/adSwarm/checkpoints/drone/1778957678487/0000000039976960.bin
```

Decode locally into an artifact directory, for example:

```bash
mkdir -p stirling/artifacts/stage1
base64 -d > stirling/artifacts/stage1/0000000039976960.bin
```

Then record the local artifact path in `stirling/stage1_baseline.md` or the
current baseline notes.

### 10. Terminate the pod

After the checkpoint and logs are copied, terminate the pod and verify RunPod
spend is zero. The successful run used a pod costing `$0.99/hr`.

## Troubleshooting Notes

- If the dashboard shows 0% GPU during clone/install/build, that is expected.
  GPU utilization begins only once `puffer train drone` starts.
- If `git clone` appears stuck at `git index-pack`, stop it and retry with the
  shallow branch clone above.
- If `build.sh drone` says `clang: command not found`, install `clang`.
- If `build.sh drone` says `omp.h file not found`, install `libomp-dev`.
- If `build.sh drone` says `ccache: command not found`, install `ccache`.
- If `puffer train drone` fails importing `_C`, the backend did not build; rerun
  `bash build.sh drone` and do not start training until the import check passes.
