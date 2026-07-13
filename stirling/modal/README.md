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

The checkpoint lands at `stirling/artifacts/drone/<tag>/<step>.bin`. Eval it
on your Mac:

```bash
bash stirling/scripts/eval_stage1_macos.sh stirling/artifacts/drone/<tag>/<step>.bin
```

## How it works

- **Runtime build, cached.** The native CUDA backend is compiled on the
  first GPU call (where a real GPU is present, so nvcc `-arch=native` and
  all driver libs resolve). The compiled `pufferlib._C` is cached to a Modal
  Volume keyed by a hash of the C/CUDA sources — change the env code and the
  next run recompiles; otherwise runs start in seconds. ccache is persisted
  too, so even a recompile is fast.
- **Checkpoint returned inline.** The ~600 KB `.bin` is returned by the
  function and written locally by the entrypoint, and also mirrored to a
  `stirling-drone-checkpoints` Volume as a durable backup.
- **Experiments are args, not rebuilds.** Reward weights, task, timesteps,
  agent count, etc. are `puffer` CLI overrides passed through `--extra`, so
  they never trigger a recompile.

## Volumes created

- `stirling-drone-build` — compiled backend cache + ccache
- `stirling-drone-checkpoints` — checkpoint backups per `--tag`

Inspect or fetch from them with `modal volume ls` / `modal volume get`.
