"""Layer-2 formation manager (tech doc v0.6 §4.2–4.3).

Python reconstruction of the missing formation_manager.m: slot-offset
lookup table for the 5 modes, heading rotation Rz(yaw) x offset, and the
linear transition blend that keeps targets moving smoothly so the
tracker converges inside the 2 s reform window (§4.4).

Box/square is HOME — the competition default. Line, stack, compressed,
and diamond are transient obstacle-traversal deviations.
"""

from dataclasses import dataclass

import numpy as np

MODES = ("box", "line", "stack", "compressed", "diamond")
N_DRONES = 4


def slot_offsets(mode, fp):
    """(4, 3) formation-frame slot offsets for a mode.

    Slot order is chosen so box<->deviation transitions do not cross
    paths: slot 0 front-left, 1 front-right, 2 rear-right, 3 rear-left
    (x forward, y left).
    """
    s = fp.box_side / 2.0
    if mode == "box":
        return np.array([[ s,  s, 0], [ s, -s, 0], [-s, -s, 0], [-s,  s, 0]], float)
    if mode == "compressed":
        c = s * fp.compressed_scale
        return np.array([[ c,  c, 0], [ c, -c, 0], [-c, -c, 0], [-c,  c, 0]], float)
    if mode == "line":
        d = fp.line_spacing
        return np.array([[1.5 * d, 0, 0], [0.5 * d, 0, 0],
                         [-0.5 * d, 0, 0], [-1.5 * d, 0, 0]], float)
    if mode == "stack":
        d = fp.stack_spacing
        return np.array([[0, 0, 1.5 * d], [0, 0, 0.5 * d],
                         [0, 0, -0.5 * d], [0, 0, -1.5 * d]], float)
    if mode == "diamond":
        return np.array([[fp.diamond_long, 0, 0], [0, -fp.diamond_wide, 0],
                         [-fp.diamond_long, 0, 0], [0, fp.diamond_wide, 0]], float)
    raise ValueError(f"unknown mode {mode!r}")


def rz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


@dataclass
class Centroid:
    """Virtual formation centre: position, velocity, yaw (course heading)."""
    p: np.ndarray
    v: np.ndarray
    yaw: float = 0.0
    yaw_rate: float = 0.0


class FormationManager:
    """Holds current/previous mode and the blend clock."""

    def __init__(self, fp, mode="box"):
        self.fp = fp
        self.mode = mode
        self.prev_mode = mode
        self.blend_t0 = -np.inf

    def set_mode(self, mode, t):
        if mode == self.mode:
            return
        self.prev_mode = self.mode
        self.mode = mode
        self.blend_t0 = t

    def blend_alpha(self, t):
        return float(np.clip((t - self.blend_t0) / self.fp.blend_time, 0.0, 1.0))

    def targets(self, centroid, t):
        """World-frame (positions (4,3), velocities (4,3)) for all slots.

        target = centroid.p + Rz(yaw) x offset_blend(t)
        v_target includes the centroid velocity, the blend rate, and the
        yaw-rate term so feedforward stays exact on a turning course.
        """
        a = self.blend_alpha(t)
        off_a = slot_offsets(self.prev_mode, self.fp)
        off_b = slot_offsets(self.mode, self.fp)
        off = (1 - a) * off_a + a * off_b
        r = rz(centroid.yaw)

        p_t = centroid.p + off @ r.T

        d_off = np.zeros_like(off)
        if 0.0 < a < 1.0:
            d_off = (off_b - off_a) / self.fp.blend_time
        omega = np.array([0.0, 0.0, centroid.yaw_rate])
        v_t = centroid.v + d_off @ r.T + np.cross(omega, off @ r.T)
        return p_t, v_t
