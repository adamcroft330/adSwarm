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

## Exact Replication Steps

These steps reproduce the successful run on a fresh RunPod secure-cloud pod.

### 1. Provision the pod

Use a secure-cloud pod. Community-cloud pods were unreliable during bring-up.

Recommended RunPod settings:

- `cloudType`: `SECURE`
- `imageName`: `runpod/pytorch:1.0.3-cu1290-torch291-ubuntu2204`
- `gpuCount`: `1`
- `containerDiskInGb`: `30`
- `volumeInGb`: `0`
- `ports`: `["8888/http", "22/tcp"]`
- `supportPublicIp`: `true`
- `gpuTypeIds`: broad list such as RTX 5090, RTX 4090, RTX A6000, RTX A5000,
  L40, A100 80GB PCIe

The successful pod used an RTX 5090. The image already had CUDA 12.9, Torch
2.9.1, `nvcc`, `git`, and `/usr/local/bin/python`.

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

This run was executed without W&B because proxy-SSH command echoing made secret
injection unsafe. `~/.wandb_key` was validated beforehand against W&B, but the
key was not placed on the pod.

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
