"""Render a trained drone checkpoint to an mp4 on Modal (headless GPU).

Runs the *native* eval path (`puffer eval drone` on the flat .bin — the same
CUDA backend that trained it, no torch conversion) inside a virtual X display,
and screen-records it. Use this to sanity-check a policy visually without a
local display, and to tell whether a bad-looking local (.pt) eval is the
policy or the .bin->.pt conversion:

    modal run stirling/modal/eval_video.py --tag smoke2 --seconds 20

The mp4 lands at stirling/artifacts/drone/<tag>/<tag>_native.mp4.
"""

import os
import pathlib
import subprocess
import sys
import time

import modal

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2] if len(
    pathlib.Path(__file__).resolve().parents) > 2 else pathlib.Path(__file__).resolve().parent
REMOTE_ROOT = "/root/adSwarm"
BUILD_DIR = "/cache"
CKPT_DIR = "/checkpoints"

# Same base as training (so the cached native _C backend is reused), plus a
# virtual X display, software OpenGL (llvmpipe), the X11/GL runtime libs the
# linux raylib build links against, and ffmpeg to record the display.
image = (
    modal.Image.from_registry(
        "nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04", add_python="3.11"
    )
    .apt_install(
        "clang", "libomp-dev", "ccache", "git", "build-essential", "curl", "unzip",
        # headless rendering + recording
        "xvfb", "ffmpeg",
        "libgl1-mesa-dri", "libgl1-mesa-glx", "libglu1-mesa",
        "libx11-6", "libxrandr2", "libxinerama1", "libxcursor1", "libxi6",
    )
    .pip_install("torch>=2.9", index_url="https://download.pytorch.org/whl/cu128")
    .pip_install("numpy", "rich", "rich_argparse", "gpytorch",
                 "scikit-learn", "wandb", "pybind11", "setuptools")
    .add_local_dir(
        REPO_ROOT, REMOTE_ROOT, copy=True,
        ignore=[".git/**", ".venv*/**", "checkpoints/**", "wandb/**",
                "**/__pycache__/**", "stirling/artifacts/**", "build/**",
                "*.mp4", "*.so", "*.o", "*.a", "**/.DS_Store", ".DS_Store"],
    )
    .workdir(REMOTE_ROOT)
    .run_commands("python -m pip install -e . --no-deps")
)

app = modal.App("stirling-drone-eval-video")
build_cache = modal.Volume.from_name("stirling-drone-build", create_if_missing=True)
checkpoints = modal.Volume.from_name("stirling-drone-checkpoints", create_if_missing=True)


def _source_hash() -> str:
    import hashlib
    h = hashlib.sha256()
    globs = ["ocean/drone/*.c", "ocean/drone/*.h", "src/*.cu", "src/*.h", "build.sh"]
    for g in globs:
        for p in sorted(pathlib.Path(REMOTE_ROOT).glob(g)):
            h.update(p.name.encode())
            h.update(p.read_bytes())
    return h.hexdigest()[:16]


def _ensure_backend(float32=True):
    """Restore/compile the native _C. Defaults to fp32 to match fp32-trained
    checkpoints (training defaults to fp32) and so the torch backend, which
    requires fp32, can run. float32=False builds bf16 for bf16 checkpoints."""
    import glob
    import shutil
    import sysconfig

    src_hash = _source_hash()
    suffix = "_f32" if float32 else ""
    cached = os.path.join(BUILD_DIR, f"_C_{src_hash}{suffix}.so")
    ext = sysconfig.get_config_var("EXT_SUFFIX")
    dest = os.path.join(REMOTE_ROOT, "pufferlib", f"_C{ext}")
    if os.path.exists(cached):
        print(f"[backend] cache hit ({src_hash}{suffix}) — restoring compiled _C", flush=True)
        shutil.copy(cached, dest)
        return
    print(f"[backend] cache miss ({src_hash}{suffix}) — compiling native CUDA backend", flush=True)
    env = os.environ.copy()
    env["NVCC_ARCH"] = "native"
    env["CCACHE_DIR"] = os.path.join(BUILD_DIR, "ccache")
    env["CCACHE_TEMPDIR"] = "/tmp/ccache-tmp"
    env["CCACHE_NOHARDLINK"] = "1"
    env["PYTHON"] = "python"
    os.makedirs(env["CCACHE_DIR"], exist_ok=True)
    os.makedirs(env["CCACHE_TEMPDIR"], exist_ok=True)
    cmd = ["bash", "build.sh", "drone"] + (["--float"] if float32 else [])
    subprocess.run(cmd, cwd=REMOTE_ROOT, env=env, check=True)
    built = glob.glob(os.path.join(REMOTE_ROOT, "pufferlib", "_C*.so"))
    shutil.copy(built[0], cached)
    build_cache.commit()


@app.function(image=image, gpu="A10G",
              volumes={BUILD_DIR: build_cache, CKPT_DIR: checkpoints}, timeout=1200)
def render_eval(tag="smoke2", seconds=20, warmup=6, extra="", bf16=False):
    """Screen-record native eval of the newest .bin under <tag> to an mp4.

    Defaults to an fp32 native backend to match fp32-trained checkpoints; pass
    bf16=True to render a bf16-trained checkpoint in its own precision.
    """
    import glob
    import shlex
    import signal

    _ensure_backend(float32=not bf16)

    bins = sorted(glob.glob(os.path.join(CKPT_DIR, tag, "*.bin")))
    if not bins:
        raise RuntimeError(f"no .bin checkpoint under {CKPT_DIR}/{tag}")
    ckpt = bins[-1]
    print(f"[video] rendering {ckpt}", flush=True)

    env = os.environ.copy()
    env["DISPLAY"] = ":99"
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GALLIUM_DRIVER"] = "llvmpipe"
    env["WANDB_MODE"] = "disabled"

    # 1) virtual display sized to the raylib window (1080x720)
    xvfb = subprocess.Popen(["Xvfb", ":99", "-screen", "0", "1080x720x24", "-ac"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    # 2) native eval (renders into the virtual display); log to a file
    cmd = ["puffer", "eval", "drone", "--load-model-path", ckpt]
    if extra:
        cmd += shlex.split(extra)
    logf = open("/tmp/eval.log", "wb")
    evalp = subprocess.Popen(cmd, cwd=REMOTE_ROOT, env=env,
                             stdout=logf, stderr=subprocess.STDOUT)

    # 3) let the window appear (backend restore + weight load + first frames)
    time.sleep(warmup)
    if evalp.poll() is not None:
        logf.close()
        tail = pathlib.Path("/tmp/eval.log").read_bytes()[-3000:].decode("utf-8", "replace")
        xvfb.kill()
        raise RuntimeError(f"eval exited early (code {evalp.returncode}). log:\n{tail}")

    # 4) record the display
    out = "/tmp/out.mp4"
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-f", "x11grab", "-video_size", "1080x720",
         "-framerate", "30", "-i", ":99", "-t", str(seconds),
         "-pix_fmt", "yuv420p", "-c:v", "libx264", "-preset", "veryfast", out],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    ff.wait()

    # 5) tear down
    evalp.send_signal(signal.SIGINT)
    time.sleep(1)
    evalp.kill()
    xvfb.kill()
    logf.close()

    tail = pathlib.Path("/tmp/eval.log").read_bytes()[-1500:].decode("utf-8", "replace")
    print("[video] eval log tail:\n" + tail, flush=True)
    if not os.path.exists(out) or os.path.getsize(out) < 2000:
        raise RuntimeError("no/empty video produced. eval log tail:\n" + tail)

    data = pathlib.Path(out).read_bytes()
    print(f"[video] recorded {len(data)//1024} KB", flush=True)
    return {"filename": f"{tag}_native.mp4", "bytes": data, "log": tail, "ckpt": os.path.basename(ckpt)}


def _eval_args(P, sys, tag_pt=None):
    argv, sys.argv = sys.argv, [sys.argv[0]]
    try:
        a = P.load_config("drone")
    finally:
        sys.argv = argv
    a["reset_state"] = False
    a["train"]["horizon"] = 1
    if tag_pt:
        a["load_model_path"] = tag_pt
    return a


@app.function(image=image, gpu="A10G",
              volumes={BUILD_DIR: build_cache, CKPT_DIR: checkpoints}, timeout=1200)
def eval_native(tag="hover-40m", steps=1500, float32=True):
    """Native (.bin) eval on its own GPU/CUDA context; return env log."""
    import glob
    import sys
    _ensure_backend(float32=float32)
    sys.path.insert(0, REMOTE_ROOT)
    os.chdir(REMOTE_ROOT)
    import pufferlib.pufferl as P
    import pufferlib._C as _C
    ckpt = sorted(glob.glob(os.path.join(CKPT_DIR, tag, "*.bin")))[-1]
    a = _eval_args(P, sys)
    p = _C.create_pufferl(a)
    _C.load_weights(p, ckpt)
    for _ in range(steps):
        _C.rollouts(p)
    log = dict(_C.eval_log(p).get("env", {}))
    return {"log": log, "ckpt": os.path.basename(ckpt), "precision": "fp32" if float32 else "bf16"}


@app.function(image=image, gpu="A10G",
              volumes={BUILD_DIR: build_cache, CKPT_DIR: checkpoints}, timeout=1200)
def eval_torch(tag="hover-40m", steps=1500):
    """Torch (.pt) eval on its own GPU/CUDA context; return env log."""
    import glob
    import sys
    _ensure_backend(float32=True)  # torch backend requires fp32 _C
    sys.path.insert(0, REMOTE_ROOT)
    os.chdir(REMOTE_ROOT)
    import pufferlib.pufferl as P
    import pufferlib.torch_pufferl as T
    ckpt = sorted(glob.glob(os.path.join(CKPT_DIR, tag, "*.pt")))[-1]
    a = _eval_args(P, sys, tag_pt=ckpt)
    p = T.PuffeRL.create_pufferl(a)
    T.PuffeRL.load_weights(p, ckpt)
    for _ in range(steps):
        p.rollouts()
    log = dict(p.log().get("env", {}))
    return {"log": log, "ckpt": os.path.basename(ckpt), "precision": "fp32"}


@app.local_entrypoint()
def metrics(tag: str = "hover-40m", steps: int = 1500):
    # separate containers → separate CUDA contexts (native _C and torch can't share one)
    nf = eval_native.spawn(tag=tag, steps=steps, float32=True)
    tf = eval_torch.spawn(tag=tag, steps=steps)
    n, t = nf.get(), tf.get()
    keys = ["ema_dist", "ema_vel", "ema_omega", "perf", "score", "episode_length", "episode_return"]
    print(f"\n=== eval metrics on Modal GPU — {n['ckpt']} (both fp32) ===")
    print(f"{'metric':16s} {'native (.bin)':>16s} {'torch (.pt)':>16s}")
    for k in keys:
        nv = n["log"].get(k); tv = t["log"].get(k)
        ns = f"{nv:.4f}" if isinstance(nv, (int, float)) else str(nv)
        ts = f"{tv:.4f}" if isinstance(tv, (int, float)) else str(tv)
        print(f"{k:16s} {ns:>16s} {ts:>16s}")


@app.local_entrypoint()
def main(tag: str = "smoke2", seconds: int = 20, warmup: int = 6, extra: str = "",
         bf16: bool = False):
    result = render_eval.remote(tag=tag, seconds=seconds, warmup=warmup, extra=extra, bf16=bf16)
    out_dir = REPO_ROOT / "stirling" / "artifacts" / "drone" / tag
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / result["filename"]
    out_path.write_bytes(result["bytes"])
    print(f"\n✅ native eval video ({result['ckpt']}): {out_path.relative_to(REPO_ROOT)}")
    print(f"   open it: open {out_path.relative_to(REPO_ROOT)}")
