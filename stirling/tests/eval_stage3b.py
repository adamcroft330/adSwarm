#!/usr/bin/env python3
"""Headless Stage 3b evaluation (stage3_plan.md task g).

    python stirling/tests/eval_stage3b.py            # full comparison table
    python stirling/tests/eval_stage3b.py --steps 2048

Answers: does the RL residual beat the classical floor, and how does either
compare to unconstrained motor-level control?

Why this exists rather than a C harness: the native `puffernet.h` weight
loader disagrees with the torch policy that was actually trained (it aligns
tensor offsets to 8 floats, but the exported checkpoint is densely packed, so
every MinGRU weight after the 4-element logstd is read shifted). A C eval
therefore runs a *different function* than the one that trained. This drives
the same C environment through the torch policy instead, which is ground
truth by construction.

Every metric here is computed inside `c_step` and surfaced through `my_log` —
the same code path `bench_stage3a.c` reports, so the numbers are directly
comparable rather than a reimplementation.

The classical floor is measured through this exact harness too (k_res=0, the
policy loaded but its output discarded by the controller). That removes any
harness difference between floor and residual, and it self-checks: the floor
row must reproduce bench_stage3a.c, and if it does not, this harness is wrong.

Actions are the distribution MEAN, not a sample — exploration off, measuring
what the policy learned.
"""
import argparse
import glob
import os
import sys

import torch

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from pufferlib import _C  # noqa: E402
from pufferlib.pufferl import load_config  # noqa: E402
from pufferlib.torch_pufferl import PuffeRL, _actions_for_vec_step, load_policy  # noqa: E402

ART = os.path.join(REPO, "stirling", "artifacts", "drone")

# FORMATION, box-only, at the shipped cruise speed — the config the floor was
# measured on (see progress_log.md). num-drones 4 is required: one env holds
# one formation.
BASE_ENV = [
    "--env.task", "2",
    "--env.num-drones", "4",
    "--env.separation-floor", "0.4",
]


def checkpoint(tag):
    hits = sorted(glob.glob(os.path.join(ART, tag, "*.pt")))
    return hits[-1] if hits else None


def evaluate(ckpt, k_res, control_mode, steps):
    """Run one config headlessly; return the env's own aggregated log."""
    overrides = BASE_ENV + [
        "--env.control-mode", str(control_mode),
        "--env.k-res", str(k_res),
    ]
    argv, sys.argv = sys.argv, ["puffer"] + overrides
    try:
        args = load_config("drone")
    finally:
        sys.argv = argv

    args["reset_state"] = False   # carry recurrent state across steps
    args["train"]["horizon"] = 1  # we drive the loop ourselves

    # Built directly rather than via create_pufferl so verbose=False suppresses
    # the training dashboard.
    args["vec"]["num_buffers"] = 1
    vec_ = _C.create_vec(args, _C.gpu)
    policy_ = load_policy(args, vec_)
    pufferl = PuffeRL(args, vec_, policy_, verbose=False)
    if ckpt:
        pufferl.load_weights(ckpt)
    policy, vec = pufferl.policy, pufferl._vec
    device = pufferl.device

    state = policy.initial_state(pufferl.total_agents, device)
    obs = pufferl.vec_obs
    with torch.no_grad():
        for _ in range(steps):
            logits, _value, state = policy.forward_eval(
                torch.as_tensor(obs, device=device), state)
            # Deterministic: the Normal's mean, not a sample.
            action = logits.loc if isinstance(logits, torch.distributions.Normal) \
                else logits.argmax(dim=-1)
            vec.cpu_step(_actions_for_vec_step(action.view(pufferl.total_agents, -1)).data_ptr())
            obs, done = pufferl.vec_obs, pufferl.vec_terminals
            # Zero hidden state for agents whose episode just ended.
            keep = (1.0 - done.float()).view(1, -1, 1)
            state = tuple(s * keep for s in state)

    log = dict(vec.log())
    vec.close()
    return log


ROWS = [
    # label,                    tag,                     k_res, control_mode
    ("classical floor (k_res=0)", "stage3b-kres025",       0.0,  1),
    ("residual k_res=0.25",       "stage3b-kres025",       0.25, 1),
    ("residual k_res=0.5",        "stage3b-kres050",       0.5,  1),
    ("residual k_res=1.0",        "stage3b-kres100",       1.0,  1),
    ("motor ceiling",             "stage3-ceiling-motor",  0.0,  0),
]

FIELDS = ["score", "perf", "ema_dist", "ema_vel", "episode_length", "oob",
          "collisions", "sep_breach"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2048,
                    help="env steps per config (2048 = 2 horizons)")
    opts = ap.parse_args()

    # Metrics are only emitted when episodes *end* (add_log runs on reset), so
    # a run shorter than HORIZON silently reports an empty table.
    if opts.steps < 1024:
        print(f"WARNING: --steps {opts.steps} is below HORIZON (1024). Episodes "
              "will not complete, so the env log stays empty and every metric "
              "reads '-'. Use >= 1024.\n")

    print(f"Stage 3b evaluation — FORMATION, box-only, {opts.steps} steps per config.")
    print("Deterministic (mean) actions. Metrics come from the env's own log,")
    print("the same path bench_stage3a.c reports.\n")
    print("The floor row runs the SAME harness with the residual disabled, so it")
    print("must reproduce bench_stage3a.c (score 935.88, ema_dist 0.0578). If it")
    print("does not, distrust this table rather than the controller.\n")

    hdr = f"{'config':<28}" + "".join(f"{f:>15}" for f in FIELDS)
    print(hdr)
    print("-" * len(hdr))

    for label, tag, k_res, cm in ROWS:
        ckpt = checkpoint(tag)
        if ckpt is None:
            print(f"{label:<28}  MISSING checkpoint ({tag})")
            continue
        log = evaluate(ckpt, k_res, cm, opts.steps)
        cells = "".join(
            f"{log.get(f, float('nan')):>15.4f}" if isinstance(log.get(f), float)
            else f"{log.get(f, '-'):>15}" for f in FIELDS)
        print(f"{label:<28}{cells}")


if __name__ == "__main__":
    main()
