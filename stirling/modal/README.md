# Modal Training — drone env

One command trains the PufferLib drone env on a cloud GPU and drops the
checkpoint straight back onto your machine. No SSH, no provisioning or
teardown, no file copying.

## One-time setup

```bash
pip install modal
modal token new          # opens a browser, authorises this machine
```

Optional — Weights & Biases logging:

```bash
modal secret create stirling-wandb WANDB_API_KEY=<your-key>
```

## Run

```bash
# Full HOVER baseline (config/drone.ini defaults: 40M steps)
modal run stirling/modal/train_drone.py

# Quick smoke run
modal run stirling/modal/train_drone.py --timesteps 2000000 --tag smoke

# Pick a GPU and log to W&B
modal run stirling/modal/train_drone.py --gpu L4 --wandb

# Arbitrary experiment overrides (any puffer arg via --extra)
modal run stirling/modal/train_drone.py --tag rw-sweep \
    --extra "--env.alpha-dist 1.5 --env.alpha-omega 0.005"
```

Flags: `--timesteps`, `--agents`, `--tag`, `--gpu` (A10G/L4/A100/H100),
`--extra "<puffer args>"`, `--wandb`.

Two checkpoint files land in `stirling/artifacts/drone/<tag>/`:

- `<step>.bin` — the native flat-float32 checkpoint the CUDA backend trains
  and saves. Only the native backend can load it.
- `<step>.pt` — the same weights converted to a torch state_dict (losslessly,
  on the GPU right after training). This is the one you eval on a Mac:

```bash
source .venv-macos-eval/bin/activate
puffer eval drone --slowly --load-model-path stirling/artifacts/drone/<tag>/<step>.pt
```

(`--slowly` selects the PyTorch backend, which renders via Raylib and loads
torch checkpoints on CPU. To convert an older native `.bin` by hand:
`python stirling/scripts/convert_native_checkpoint.py <path>.bin`.)

## How it works

- **Runtime build, cached.** The native CUDA backend is compiled on the
  first GPU call (where a real GPU is present, so nvcc `-arch=native` and
  all driver libs resolve). The compiled `pufferlib._C` is cached to a Modal
  Volume keyed by a hash of the C/CUDA sources — change the env code and the
  next run recompiles; otherwise runs start in seconds. ccache is persisted
  too, so even a recompile is fast.
- **Checkpoints returned inline.** The ~600 KB `.bin` and its `.pt`
  conversion are returned by the function and written locally by the
  entrypoint, and also mirrored to a `stirling-drone-checkpoints` Volume as
  a durable backup.
- **Native speed, Mac eval.** Training stays on the fast native CUDA
  backend; `stirling/scripts/convert_native_checkpoint.py` maps its flat
  weight dump onto the torch policy's state_dict (the layouts match
  one-to-one; torch-only biases are zeroed, which is the identical
  function), so `puffer eval drone --slowly` works on a Mac with no GPU.
- **Experiments are args, not rebuilds.** Reward weights, task, timesteps,
  agent count, etc. are `puffer` CLI overrides passed through `--extra`, so
  they never trigger a recompile.

## Volumes created

- `stirling-drone-build` — compiled backend cache + ccache
- `stirling-drone-checkpoints` — checkpoint backups per `--tag`

Inspect or fetch from them with `modal volume ls` / `modal volume get`.
