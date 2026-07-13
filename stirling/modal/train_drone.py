"""Ultra-low-friction Modal training for the PufferLib drone env.

One command trains the drone env on a cloud GPU and drops the resulting
checkpoint straight back onto your machine — no SSH, no manual
provisioning or teardown, no file copying.

    modal run stirling/modal/train_drone.py                       # full HOVER baseline
    modal run stirling/modal/train_drone.py --timesteps 5_000_000 --tag quicktest
    modal run stirling/modal/train_drone.py --gpu L4 --wandb
    modal run stirling/modal/train_drone.py --extra "--env.alpha-dist 1.5 --env.task 1"

The trained checkpoint lands in stirling/artifacts/drone/<tag>/ locally,
ready to eval on your Mac:

    bash stirling/scripts/eval_stage1_macos.sh stirling/artifacts/drone/<tag>/<step>.bin

One-time setup:  pip install modal  &&  modal token new
Optional W&B:    modal secret create stirling-wandb WANDB_API_KEY=<key>   (then pass --wandb)

Design notes
------------
The native CUDA backend is compiled *at runtime* on the first GPU call,
where a real GPU is present so nvcc's `-arch=native` autodetect and every
driver lib resolve correctly. The compiled `_C` extension is cached to a
Modal Volume keyed by a hash of the C/CUDA sources: change the env code
and the next run recompiles; otherwise every run reuses the cache and
starts training in seconds. ccache is persisted too, so even a recompile
is fast.
"""

import hashlib
import os
import pathlib
import subprocess
import sys
import time

import modal

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
REMOTE_ROOT = "/root/adSwarm"

# --- Container image ---------------------------------------------------------
# CUDA *devel* base (ships nvcc + cuDNN); torch cu128 wheel matches the 12.8
# toolkit. The env source is baked in so the runtime build has everything it
# needs; the heavy CUDA compile itself happens at runtime (see module docstring).
image = (
    modal.Image.from_registry(
        "nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04", add_python="3.11"
    )
    .apt_install(
        "clang", "libomp-dev", "ccache", "git", "build-essential",
        "libnccl2", "libnccl-dev", "curl", "unzip",
    )
    .pip_install("torch>=2.9", index_url="https://download.pytorch.org/whl/cu128")
    .pip_install(
        "numpy", "rich", "rich_argparse", "gpytorch",
        "scikit-learn", "wandb", "pybind11", "setuptools",
    )
    .add_local_dir(
        REPO_ROOT,
        REMOTE_ROOT,
        copy=True,
        ignore=[
            ".git/**", ".venv*/**", "checkpoints/**", "wandb/**",
            "**/__pycache__/**", "stirling/artifacts/**", "build/**",
            "*.mp4", "*.so", "*.o", "*.a",
        ],
    )
    .workdir(REMOTE_ROOT)
    # Editable install only registers the Python package; it does NOT compile
    # the C backend (build.sh does, at runtime).
    .run_commands("python -m pip install -e . --no-deps")
)

app = modal.App("stirling-drone-train")

build_cache = modal.Volume.from_name("stirling-drone-build", create_if_missing=True)
checkpoints = modal.Volume.from_name("stirling-drone-checkpoints", create_if_missing=True)

BUILD_DIR = "/cache"
CKPT_DIR = "/checkpoints"


def _source_hash() -> str:
    """Hash the C/CUDA sources that determine the compiled backend."""
    h = hashlib.sha256()
    globs = ["ocean/drone/*.c", "ocean/drone/*.h", "src/*.cu", "src/*.h", "build.sh"]
    paths = []
    for g in globs:
        paths.extend(sorted(pathlib.Path(REMOTE_ROOT).glob(g)))
    for p in paths:
        h.update(p.name.encode())
        h.update(p.read_bytes())
    return h.hexdigest()[:16]


def _ensure_backend():
    """Compile pufferlib._C for the drone env, or restore it from the cache."""
    import glob
    import shutil

    src_hash = _source_hash()
    cached = os.path.join(BUILD_DIR, f"_C_{src_hash}.so")

    # The extension filename (e.g. _C.cpython-311-x86_64-linux-gnu.so).
    import sysconfig
    ext = sysconfig.get_config_var("EXT_SUFFIX")
    dest = os.path.join(REMOTE_ROOT, "pufferlib", f"_C{ext}")

    if os.path.exists(cached):
        print(f"[backend] cache hit ({src_hash}) — restoring compiled _C", flush=True)
        shutil.copy(cached, dest)
        return

    print(f"[backend] cache miss ({src_hash}) — compiling native CUDA backend", flush=True)
    env = os.environ.copy()
    env["NVCC_ARCH"] = "native"          # a real GPU is attached at runtime
    env["CCACHE_DIR"] = os.path.join(BUILD_DIR, "ccache")
    env["PYTHON"] = "python"
    os.makedirs(env["CCACHE_DIR"], exist_ok=True)
    subprocess.run(["bash", "build.sh", "drone"], cwd=REMOTE_ROOT, env=env, check=True)

    built = glob.glob(os.path.join(REMOTE_ROOT, "pufferlib", "_C*.so"))
    if not built:
        raise RuntimeError("build.sh finished but no pufferlib/_C*.so was produced")
    shutil.copy(built[0], cached)
    build_cache.commit()
    print(f"[backend] compiled and cached as {os.path.basename(cached)}", flush=True)


@app.function(
    image=image,
    gpu="A10G",
    volumes={BUILD_DIR: build_cache, CKPT_DIR: checkpoints},
    timeout=3600,
)
def train(timesteps=None, agents=None, tag="drone", extra="", wandb_enabled=False):
    """Run one training job and return the newest checkpoint as bytes."""
    import glob
    import shlex
    import shutil

    _ensure_backend()

    # Verify the backend imports before spending GPU time on a doomed run.
    subprocess.run(
        ["python", "-c", "import pufferlib, pufferlib._C; print('pufferlib + _C import OK')"],
        cwd=REMOTE_ROOT, check=True,
    )

    cmd = ["puffer", "train", "drone", "--tag", tag]
    if timesteps is not None:
        cmd += ["--train.total-timesteps", str(int(timesteps))]
    if agents is not None:
        cmd += ["--vec.total-agents", str(int(agents))]

    env = os.environ.copy()
    if wandb_enabled and env.get("WANDB_API_KEY"):
        cmd += ["--wandb", "--wandb-project", "stirling-drone", "--wandb-group", tag]
    else:
        env["WANDB_MODE"] = "disabled"
        if wandb_enabled:
            print("[wandb] requested but no WANDB_API_KEY secret found — running disabled",
                  flush=True)
    if extra:
        cmd += shlex.split(extra)

    print(f"[train] {' '.join(cmd)}", flush=True)
    t0 = time.time()
    subprocess.run(cmd, cwd=REMOTE_ROOT, env=env, check=True)
    dt = time.time() - t0

    # Newest checkpoint under checkpoints/drone/<run_id>/<step>.bin
    bins = glob.glob(os.path.join(REMOTE_ROOT, "checkpoints", "drone", "**", "*.bin"),
                     recursive=True)
    if not bins:
        raise RuntimeError("training finished but no checkpoint .bin was written")
    newest = max(bins, key=os.path.getmtime)
    name = os.path.basename(newest)

    # Persist to the checkpoints Volume as a durable backup.
    dst_dir = os.path.join(CKPT_DIR, tag)
    os.makedirs(dst_dir, exist_ok=True)
    shutil.copy(newest, os.path.join(dst_dir, name))
    checkpoints.commit()

    data = pathlib.Path(newest).read_bytes()
    print(f"[train] done in {dt:.0f}s — checkpoint {name} ({len(data)//1024} KB)", flush=True)
    return {"filename": name, "bytes": data, "tag": tag, "seconds": dt}


@app.local_entrypoint()
def main(timesteps: int = None, agents: int = None, tag: str = "drone",
         gpu: str = "A10G", extra: str = "", wandb: bool = False):
    """Train on Modal and drop the checkpoint into stirling/artifacts/drone/<tag>/."""
    # The W&B secret is only attached (and thus only required) when --wandb is set.
    opts = {"gpu": gpu}
    if wandb:
        opts["secrets"] = [modal.Secret.from_name("stirling-wandb")]
    result = train.with_options(**opts).remote(
        timesteps=timesteps, agents=agents, tag=tag, extra=extra, wandb_enabled=wandb,
    )

    out_dir = REPO_ROOT / "stirling" / "artifacts" / "drone" / tag
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / result["filename"]
    out_path.write_bytes(result["bytes"])

    rel = out_path.relative_to(REPO_ROOT)
    print(f"\n✅ trained in {result['seconds']:.0f}s on {gpu}")
    print(f"   checkpoint saved: {rel}")
    print(f"   eval locally:     bash stirling/scripts/eval_stage1_macos.sh {rel}")
