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

# Faster bf16 training — ONLY if you'll eval on Modal, not the Mac (see Precision)
modal run stirling/modal/train_drone.py --tag fast-exp --bf16

# Arbitrary experiment overrides (any puffer arg via --extra)
modal run stirling/modal/train_drone.py --tag rw-sweep \
    --extra "--env.alpha-dist 1.5 --env.alpha-omega 0.005"
```

Flags: `--timesteps`, `--agents`, `--tag`, `--gpu` (A10G/L4/A100/H100),
`--extra "<puffer args>"`, `--wandb`, `--bf16` (see [Precision](#precision)).

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

If eval aborts with `OMP Error #15` or segfaults in libomp, your local `_C`
was built before build.sh learned to retarget libomp to torch's bundled copy
— rebuild it: `CC=/opt/homebrew/opt/llvm/bin/clang bash build.sh drone --cpu`.
Do **not** work around it with `KMP_DUPLICATE_LIB_OK=TRUE`; two OpenMP
runtimes in one process segfault sooner or later.

## Precision

**Training defaults to fp32, on purpose.** The native CUDA backend can train in
either bf16 (faster) or fp32, but the Mac/torch eval path (`--slowly`) is
**fp32-only**. A bf16-trained policy — at least this recurrent hover controller
— does **not** transfer to fp32: it hovers under bf16 but destabilises within
~100 steps under fp32. So a bf16 checkpoint looks perfect when rendered on Modal
(native bf16) yet falls apart on the Mac, with *identical weights*. That is not
a conversion bug — native-fp32 and torch-fp32 agree to 3 decimals; the policy is
simply numerically fragile across precisions.

To keep "trains on Modal, evals on the Mac" honest, training and the native
eval/video default to **fp32**. Only pass `--bf16` if you'll eval exclusively on
Modal (and then render with `modal run stirling/modal/eval_video.py --tag <t>
--bf16` so the native renderer matches). For a model this small (~151K params,
600 KB) fp32 costs nothing at inference and is *more* portable; the only cost is
~40% slower training (e.g. 40M steps: 32s bf16 vs 45s fp32 — trivial here).

Sanity-check any checkpoint's hover quality on the GPU without a display:

```bash
modal run stirling/modal/eval_video.py::metrics --tag <tag>   # native vs torch, both fp32
```

Good hover ≈ `ema_dist` near `hover_dist` (0.1) with full-length episodes
(~1000); a broken/precision-mismatched policy shows `ema_dist` ~2.5 and episodes
dying at ~100 steps.

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
