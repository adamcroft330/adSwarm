"""Closed-loop 4-drone MuJoCo validation harness.

Python reconstruction of the missing demo_formation.m, upgraded from the
MATLAB draft's ideal velocity dynamics to full rigid-body physics:
settle into box -> moving-course tour through all 5 formation modes ->
disturbance kick -> reform, with pure classical control (dv=None).

Validates against the recorded Octave smoke-test targets (tech doc §8.4):
  - reform after disturbance kick   < 2.0 s   (MATLAB draft: 0.60 s)
  - min inter-drone separation      >= 0.40 m (MATLAB draft: 0.54 m)
Exits non-zero if either target is violated.

Usage:
  python3 demo_formation.py            # headless metrics run
  python3 demo_formation.py --video    # also render mp4 via ffmpeg
"""

import argparse
import json
import os
import shutil
import subprocess
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from classical_control_step import classical_control_step
from default_params import default_params
from drone_model import build_mjcf, get_drone_state
from formation_manager import Centroid, FormationManager, N_DRONES, slot_offsets
from quad_autopilot import QuadAutopilot

CTRL_DT = 0.01       # 100 Hz outer loop (matches env rate)
SIM_DT = 0.002       # 500 Hz physics / inner loop
SUBSTEPS = int(round(CTRL_DT / SIM_DT))
ALTITUDE = 2.2

REFORM_THRESH = 0.15   # formation "re-achieved" when max slot error < this [m]
REFORM_HOLD = 0.5      # ... sustained for this long [s]

ART_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "artifacts", "controller")


class Scenario:
    """Mode schedule, centroid course, and the disturbance event."""

    SCHEDULE = [(5.0, "line"), (8.0, "box"), (11.0, "stack"), (14.0, "box"),
                (17.0, "compressed"), (20.0, "box"), (23.0, "diamond"),
                (26.0, "box")]
    KICK_T = 29.0
    KICK_V = np.array([0.0, 2.5, 0.0])   # lateral shove on drone 1 [m/s]
    KICK_DRONE = 1
    T_END = 34.0

    def centroid(self, t):
        # Hold for 3 s, ramp to 1 m/s along +x by t=4, then constant.
        if t < 3.0:
            x, vx = 0.0, 0.0
        elif t < 4.0:
            vx = t - 3.0
            x = 0.5 * vx * vx
        else:
            vx = 1.0
            x = 0.5 + (t - 4.0)
        return Centroid(p=np.array([x, 0.0, ALTITUDE]),
                        v=np.array([vx, 0.0, 0.0]))

    def mode_at(self, t):
        mode = "box"
        for t_i, m in self.SCHEDULE:
            if t >= t_i:
                mode = m
        return mode


def make_model(qp, fp, seed=0):
    rng = np.random.default_rng(seed)
    home = slot_offsets("box", fp) + np.array([0.0, 0.0, ALTITUDE])
    spawns = home + rng.uniform(-0.3, 0.3, size=(N_DRONES, 3))
    model = mujoco.MjModel.from_xml_string(build_mjcf(default_params()[2], spawns,
                                                      timestep=SIM_DT))
    return model, mujoco.MjData(model)


def run(video_path=None, seed=0):
    cp, fp, qp = default_params()
    model, data = make_model(qp, fp, seed)
    manager = FormationManager(fp)
    autopilot = QuadAutopilot(qp)
    scen = Scenario()

    integs = np.zeros((N_DRONES, 3))
    n_steps = int(round(scen.T_END / CTRL_DT))

    log = {k: [] for k in ("t", "max_err", "min_sep", "errs")}
    kicked = False

    renderer, ffmpeg, cam = None, None, None
    if video_path:
        fps = 50
        render_every = int(round(1.0 / (fps * CTRL_DT)))
        renderer = mujoco.Renderer(model, height=720, width=1280)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation = 6.0, -18.0
        ffmpeg = subprocess.Popen(
            [shutil.which("ffmpeg"), "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", "1280x720", "-r", str(fps), "-i", "-", "-an",
             "-c:v", "libx264", "-preset", "medium", "-crf", "20",
             "-pix_fmt", "yuv420p", video_path],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)

    for step in range(n_steps):
        t = step * CTRL_DT
        manager.set_mode(scen.mode_at(t), t)
        centroid = scen.centroid(t)
        p_targets, v_targets = manager.targets(centroid, t)

        if not kicked and t >= scen.KICK_T:
            data.qvel[6 * scen.KICK_DRONE:6 * scen.KICK_DRONE + 3] += scen.KICK_V
            kicked = True

        states = [get_drone_state(data, i) for i in range(N_DRONES)]
        positions = np.array([s[0] for s in states])
        velocities = np.array([s[1] for s in states])

        v_cmds = []
        for i in range(N_DRONES):
            v_cmd, integs[i], _ = classical_control_step(
                i, positions, velocities, p_targets[i], v_targets[i],
                integs[i], None, [], CTRL_DT, cp)
            v_cmds.append(v_cmd)

        data.mocap_pos[:] = p_targets
        for _ in range(SUBSTEPS):
            for i in range(N_DRONES):
                p, v, R, om = get_drone_state(data, i)
                data.ctrl[4 * i:4 * i + 4] = autopilot.motor_thrusts(
                    R, v, om, v_cmds[i], yaw_sp=centroid.yaw)
            mujoco.mj_step(model, data)

        errs = np.linalg.norm(positions - p_targets, axis=1)
        seps = [np.linalg.norm(positions[i] - positions[j])
                for i in range(N_DRONES) for j in range(i + 1, N_DRONES)]
        log["t"].append(t)
        log["errs"].append(errs)
        log["max_err"].append(float(errs.max()))
        log["min_sep"].append(float(min(seps)))

        if renderer and step % render_every == 0:
            cam.lookat[:] = centroid.p
            cam.azimuth = 130.0 + 15.0 * np.sin(2 * np.pi * t / scen.T_END)
            renderer.update_scene(data, camera=cam)
            ffmpeg.stdin.write(renderer.render().tobytes())

    if renderer:
        ffmpeg.stdin.close()
        ffmpeg.wait()
        renderer.close()

    return compute_metrics(log, scen)


def _settle_time(t_arr, err_arr, t_event):
    """Time from t_event until max slot error stays < REFORM_THRESH for
    REFORM_HOLD seconds. Returns np.inf if it never settles."""
    hold_n = int(round(REFORM_HOLD / CTRL_DT))
    idx = np.searchsorted(t_arr, t_event)
    below = err_arr[idx:] < REFORM_THRESH
    run_len = 0
    for k, b in enumerate(below):
        run_len = run_len + 1 if b else 0
        if run_len >= hold_n:
            return t_arr[idx + k - hold_n + 1] - t_event
    return float("inf")


def compute_metrics(log, scen):
    t = np.array(log["t"])
    max_err = np.array(log["max_err"])
    min_sep = np.array(log["min_sep"])

    post_kick = t >= scen.KICK_T
    metrics = {
        "reform_time_after_kick_s": _settle_time(t, max_err, scen.KICK_T),
        "worst_formation_error_post_kick_m": float(max_err[post_kick].max()),
        "min_separation_overall_m": float(min_sep.min()),
        "min_separation_post_kick_m": float(min_sep[post_kick].min()),
        "mode_transition_settle_s": {
            f"t={t_i:g}s->{m}": _settle_time(t, max_err, t_i)
            for t_i, m in scen.SCHEDULE},
        "targets": {"reform_s": 2.0, "min_sep_m": 0.40},
        "matlab_draft_reference": {"reform_s": 0.60, "min_sep_m": 0.54,
                                   "worst_err_m": 0.49},
        "reform_threshold_m": REFORM_THRESH,
    }
    metrics["pass"] = bool(
        metrics["reform_time_after_kick_s"] < 2.0
        and metrics["min_separation_overall_m"] >= 0.40
        and all(np.isfinite(v) for v in
                metrics["mode_transition_settle_s"].values()))
    return metrics, log


def save_plot(log, scen, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array(log["t"])
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 6))
    ax1.plot(t, np.array(log["errs"]))
    ax1.axhline(REFORM_THRESH, color="k", ls=":", lw=1,
                label=f"reform threshold {REFORM_THRESH} m")
    ax1.set_ylabel("slot error [m]")
    ax1.legend(["drone 0", "drone 1", "drone 2", "drone 3", "threshold"],
               ncol=5, fontsize=8)
    ax2.plot(t, np.array(log["min_sep"]), color="tab:red")
    ax2.axhline(0.40, color="k", ls="--", lw=1, label="0.40 m floor")
    ax2.set_ylabel("min separation [m]")
    ax2.set_xlabel("t [s]")
    ax2.legend(fontsize=8)
    for ax in (ax1, ax2):
        for t_i, m in scen.SCHEDULE:
            ax.axvline(t_i, color="gray", lw=0.5, alpha=0.5)
        ax.axvline(scen.KICK_T, color="tab:orange", lw=1, alpha=0.8)
    ax1.set_title("Classical controller — formation tracking "
                  "(orange line = disturbance kick)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", action="store_true", help="render mp4")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(ART_DIR, exist_ok=True)
    video = os.path.join(ART_DIR, "demo_formation.mp4") if args.video else None

    metrics, log = run(video_path=video, seed=args.seed)
    try:
        save_plot(log, Scenario(), os.path.join(ART_DIR, "demo_metrics.png"))
    except ImportError:
        pass

    print(json.dumps(metrics, indent=2, default=str))
    with open(os.path.join(ART_DIR, "demo_metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, default=str)
    if video:
        print(f"video: {video}")

    if not metrics["pass"]:
        print("VALIDATION FAILED", file=sys.stderr)
        sys.exit(1)
    print("VALIDATION PASSED")


if __name__ == "__main__":
    main()
