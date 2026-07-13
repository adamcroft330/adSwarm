"""Convert a native (flat float32) puffer checkpoint into a torch state_dict.

The native CUDA backend saves checkpoints as a raw dump of its flat
`master_weights` buffer (no header). The torch backend (`puffer eval --slowly`)
expects a `torch.save`d state_dict. This script converts losslessly between
them so you can train fast on GPU with the native backend and still eval the
checkpoint on a Mac:

    python stirling/scripts/convert_native_checkpoint.py \
        stirling/artifacts/drone/<tag>/<step>.bin
    puffer eval drone --slowly --load-model-path stirling/artifacts/drone/<tag>/<step>.pt

Layout of the native flat buffer (registration order: encoder, decoder,
network — see policy_weights_create in src/models.cu):

    encoder.weight   (hidden, obs)        Linear, no bias
    decoder.weight   (num_atns+1, hidden) fused: rows 0..n-1 = action mean,
                                          last row = value head. No bias.
    decoder.logstd   (1, num_atns)        continuous only
    network.layers.i (3*hidden, hidden)   MinGRU, i in 0..num_layers-1

The torch DefaultEncoder/DefaultDecoder have biases the native net lacks;
they are set to zero, which reproduces the identical function. The MinGRU
[hidden|gate|proj] row order matches torch's chunk(3, dim=-1) exactly.
"""

import argparse
import pathlib
import sys

import numpy as np
import torch


def convert(bin_path: str, env_name: str = "drone", out_path: str | None = None) -> str:
    import pufferlib._C as _C
    from pufferlib.pufferl import load_config

    # load_config() parses sys.argv itself; shield it from this script's args.
    argv, sys.argv = sys.argv, [sys.argv[0]]
    try:
        args = load_config(env_name)
    finally:
        sys.argv = argv
    hidden = int(args["policy"]["hidden_size"])
    num_layers = int(args["policy"]["num_layers"])  # C backend truncates floats

    # Same source of truth for dims as torch_pufferl.load_policy.
    args["vec"]["num_buffers"] = 1
    vec = _C.create_vec(args, _C.gpu)
    obs = int(vec.obs_size)
    act_sizes = list(vec.act_sizes)
    continuous = sum(act_sizes) == len(act_sizes)
    if not continuous:
        raise NotImplementedError("converter currently supports continuous action envs")
    num_atns = len(act_sizes)

    flat = np.fromfile(bin_path, dtype=np.float32)
    expected = hidden * obs + (num_atns + 1) * hidden + num_atns + num_layers * 3 * hidden * hidden
    if flat.size != expected:
        raise ValueError(
            f"{bin_path}: {flat.size} floats, but config implies {expected} "
            f"(obs={obs} hidden={hidden} layers={num_layers} atns={num_atns}). "
            "Wrong env/config for this checkpoint?"
        )

    def take(n, shape):
        nonlocal flat
        t = torch.from_numpy(flat[:n].copy()).reshape(shape)
        flat = flat[n:]
        return t

    enc_w = take(hidden * obs, (hidden, obs))
    dec_fused = take((num_atns + 1) * hidden, (num_atns + 1, hidden))
    logstd = take(num_atns, (1, num_atns))
    layers = [take(3 * hidden * hidden, (3 * hidden, hidden)) for _ in range(num_layers)]

    state_dict = {
        "encoder.encoder.weight": enc_w,
        "encoder.encoder.bias": torch.zeros(hidden),
        "decoder.decoder_mean.weight": dec_fused[:num_atns].clone(),
        "decoder.decoder_mean.bias": torch.zeros(num_atns),
        "decoder.decoder_logstd": logstd,
        "decoder.value_function.weight": dec_fused[num_atns:].clone(),
        "decoder.value_function.bias": torch.zeros(1),
    }
    for i, w in enumerate(layers):
        state_dict[f"network.layers.{i}.weight"] = w

    out = out_path or str(pathlib.Path(bin_path).with_suffix(".pt"))
    torch.save(state_dict, out)
    n = sum(v.numel() for v in state_dict.values())
    print(f"converted {bin_path} -> {out} ({n} params, "
          f"obs={obs} hidden={hidden} layers={num_layers} atns={num_atns})")
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("checkpoint", help="native flat .bin checkpoint")
    p.add_argument("--env", default="drone")
    p.add_argument("--out", default=None, help="output .pt path (default: alongside input)")
    a = p.parse_args()
    if not pathlib.Path(a.checkpoint).is_file():
        sys.exit(f"not a file: {a.checkpoint}")
    convert(a.checkpoint, a.env, a.out)
