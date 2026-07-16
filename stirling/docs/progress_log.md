# Progress Log

Reverse-chronological record of significant project increments. Each entry
captures what changed, why, and what it unblocks. Complements the planning
docs (`stage3_plan.md`, `rl_pipeline_doc_v0_3.md`) which describe intent;
this records what actually landed.

---

## 2026-07-16 — Stage 3a regression: velocity wrapper proven inert (task a closed)

Ran the `control_mode=0` regression the Stage 3 plan sequences before any
formation logic lands, now that the Modal framework and the wrapper are on one
branch. **Result: pass, conclusively.**

### Method

Two 40M-step HOVER runs via Modal, identical config (fp32, A10G, `seed=42`,
`config/drone.ini` defaults), differing *only* in whether the wrapper code is
present:

| Run | Branch | Wrapper in `c_step`? |
| --- | --- | --- |
| `wrapper-regression` | `stage3a-velocity-wrapper` | yes (inert, `control_mode=0`) |
| `control-no-wrapper` | `stirling-drone` | no — code absent entirely |

### Result: byte-identical

Both runs produced **byte-for-byte identical checkpoints**
(`md5 14794a4ab236d48fe8fb2938edd3d80e`) and identical metrics to three
decimals: score 740.424, `ema_dist` 0.099, `episode_return` 45.418,
`episode_length` 997.307, `perf` 0.937. Training is seeded and deterministic,
so identical weights after 40M steps proves the executed code path is
bit-identical — `control_mode=0` cannot regress Stage 1.

### Why the control run mattered

Against the *historical* Stage 1 baseline (score 836.9, `ema_dist` 0.020) the
wrapper run looks ~12% worse — which would read as a regression. It is not:
that baseline was **bf16 on an RTX 5090**, this is **fp32 on an A10G**, so two
variables moved alongside the wrapper. The no-wrapper control lands on exactly
740.424 too, attributing the entire gap to precision + GPU and none of it to
the wrapper. `ema_dist` 0.099 also matches the documented fp32 expectation
(~0.10) from the 2026-07-15 precision finding. Comparing against the old
baseline alone would have been ambiguous at best and misleading at worst.

Fallback checkpoint kept at `stirling/artifacts/drone/wrapper-regression/`
(fp32 40M HOVER, `.bin` + Mac-evalable `.pt`); other experiment outputs are
gitignored as regenerable.

**Stage 3 task (a) is closed** — the wrapper is verified inert by construction
*and* by experiment (PR #4).

## 2026-07-15 — Precision finding: fp32 default; Stage 1 HOVER verified end-to-end

Ran the first full HOVER baselines through the Modal framework and chased down
why a checkpoint that hovered on Modal looked broken when eval'd on the Mac.

### Root cause: bf16-trained policy does not transfer to fp32

The native backend trains in bf16 by default. A bf16-trained 40M HOVER policy
hovers under bf16 (ema_dist ~0.03, full ~1000-step episodes) but destabilises
within ~100 steps under fp32 (ema_dist ~2.6) — and the Mac/torch eval path
(`--slowly`) is fp32-only. So the same weights looked good on Modal (native
bf16) and bad on the Mac (torch fp32).

- **Conversion exonerated.** A native-fp32 vs torch-fp32 comparison on the same
  GPU (`stirling/modal/eval_video.py::metrics`, separate CUDA contexts) agrees
  to three decimals — the `.bin`→`.pt` export is exact. The policy is simply
  numerically fragile across precisions (marginally stable).
- **Fix / decision:** training and native eval default to **fp32**
  (`stirling/modal/train_drone.py`, `--bf16` opts out). An fp32-trained 40M
  HOVER hovers in fp32 (ema_dist ~0.10, full episodes) and renders cleanly on
  the Mac. Documented in `stirling/modal/README.md` (Precision) and tech doc
  §5.2; robustness risk logged in tech doc §11.

### Also this session

- `stirling/modal/eval_video.py` — headless native eval → mp4 (Xvfb + llvmpipe
  + ffmpeg) and a native-vs-torch fp32 metrics entrypoint for GPU sanity checks.
- `pufferlib/torch_pufferl.py` — honor `reset_state` so the recurrent hidden
  state persists across eval rollouts (horizon=1) instead of zeroing every step,
  matching the native backend (correctness fix for the torch eval path).

## 2026-07-14 — Modal training framework + classical controller + Stage 3a wrapper

A large increment spanning training infrastructure, the classical control
baseline, and the first Stage 3 env code.

### Training infrastructure: Modal (new primary path)

`stirling/modal/` — a low-friction cloud-GPU training framework. One command
(`modal run stirling/modal/train_drone.py`) builds the native CUDA backend,
trains the PufferLib drone env on a Modal GPU, and returns the checkpoint to
the local machine. No SSH, provisioning, teardown, or manual file copying.

- **Runtime, cached build:** the native backend is compiled on the first GPU
  call (a real GPU is present, so nvcc `-arch=native` and the driver libs
  resolve) and cached to a Modal Volume keyed by a hash of the C/CUDA
  sources; only env-code changes trigger a recompile. ccache is persisted.
- **Checkpoints returned inline:** both the native `.bin` and a losslessly
  converted torch `.pt` are written to `stirling/artifacts/drone/<tag>/` and
  mirrored to a `stirling-drone-checkpoints` Volume.
- **Experiments are `puffer` args** passed via `--extra`, so reward-weight /
  timestep / task sweeps never rebuild.
- **Verified end-to-end:** smoke runs plus a full HOVER run to ~104M steps
  completed via Modal; eval video rendered (`stirling/modal/eval_video.py`).
- Quickstart: `stirling/modal/README.md`.

### Native → torch checkpoint bridge for Mac eval

`stirling/scripts/convert_native_checkpoint.py` maps the native backend's
flat float32 weight dump onto the torch policy `state_dict` (layouts match
one-to-one; torch-only biases are zeroed, an identical function). This lets a
GPU-trained checkpoint be evaluated visually on a Mac with no GPU:

```bash
puffer eval drone --slowly --load-model-path stirling/artifacts/drone/<tag>/<step>.pt
```

`--slowly` selects the PyTorch backend (Raylib render, CPU load). A macOS
libomp segfault on eval was fixed by retargeting `build.sh`'s libomp to
torch's bundled copy so `_C` and torch share a single OpenMP runtime — the
correct fix, not the `KMP_DUPLICATE_LIB_OK` workaround (which papers over two
OpenMP runtimes in one process and eventually crashes).

### Classical formation controller — reconstructed + validated (PR #1, merged)

The missing MATLAB/Octave draft (tech doc §8) was reconstructed as portable
Python in `stirling/controller/` (tracking law, APF safety filter, formation
manager, control-step composition) and validated in MuJoCo against 4 generic
250-class quads behind a PX4-like velocity inner loop. Meets the §8.4
smoke-test targets under full rigid-body physics: reform 1.29 s (< 2.0 s),
min separation 0.49 m (≥ 0.40 m). Theory writeup:
`stirling/docs/classical_controller_design.md`. This resolves the Stage 3
"classical controller source is missing" blocker (see `stage3_plan.md`).

### Stage 3a — velocity-setpoint wrapper in the env (branch, not yet merged)

`ocean/drone/velocity_controller.h` adds the four-layer velocity-setpoint
stack (classical P law → `k_res`-scaled residual hook → saturate → motor
mapping) in front of the native motor interface, gated by a new
`DroneEnv.control_mode` (default 0 = native motor path, byte-identical to
Stage 1). `control_mode=1, k_res=0` is pure classical velocity control.
Verified with `stirling/tests/test_velocity_wrapper.c` (native path runs;
classical hover settles). Lives on branch `stage3a-velocity-wrapper`;
**still to do:** open its PR, and run the "Stage 1 still scores ~836 under
`control_mode=0`" GPU regression — now straightforward via the Modal path.

### Housekeeping (PR #2, merged) + repo hygiene

macOS eval workflow and Stage 1 automation landed; June project docs
converted from `.docx` to markdown; render videos untracked (regenerable).

### Status after this increment

- **Stage 1 (HOVER baseline):** complete; now retrainable in one command via
  Modal, evaluable on a Mac.
- **Stage 2 (platform recalibration):** still blocked on hardware selection
  (`hardware_platform_shortlist.md`).
- **Stage 3:** classical controller done (a/b groundwork); velocity wrapper
  implemented (3a) pending PR + regression; FORMATION task + observation
  extension (c/d) not started.

### Next steps

1. Open the `stage3a-velocity-wrapper` PR; run its `control_mode=0`
   regression via Modal against the Stage 1 baseline.
2. Port the classical controller from Python into `velocity_controller.h`
   (stage3_plan task b) with the NFR-36 unit-test gate.
3. FORMATION task scaffolding + observation extension (tasks c/d).
