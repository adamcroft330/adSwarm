"""APF safety filter (tech doc v0.6 §8.3).

Python reconstruction of the missing safety_filter.m, APF path only:
artificial-potential-field repulsion for inter-drone separation,
obstacles, and the course boundary, with closing-rate damping on the
inter-drone term to brake fast approaches. No solver dependency — maps
directly to C. CBF-QP remains a later upgrade (§8.3), not required for
Stage 3.

The filter wraps (u_classic + k_res*dv), so its effect holds even if the
RL residual misbehaves.
"""

import numpy as np

from formation_tracking import clip_norm


def _ramp(d, d_act, d_floor, v_rep_max):
    """Repulsion magnitude: 0 at d_act, v_rep_max at d_floor, keeps
    growing linearly below the floor so violations are pushed out hard."""
    return v_rep_max * (d_act - d) / (d_act - d_floor)


def safety_filter(u, i, positions, velocities, obstacles, cp):
    """Filter drone i's commanded velocity setpoint.

    u          : commanded world velocity setpoint (3,) = u_classic + k_res*dv
    i          : index of this drone
    positions  : (N, 3) world positions of all drones
    velocities : (N, 3) world velocities of all drones
    obstacles  : iterable of (center(3,), radius) spheres; [] for none
    cp         : ControllerParams

    Returns the filtered, saturated setpoint (3,).
    """
    p_i = positions[i]
    v_i = velocities[i]
    u_out = u.copy()

    # Inter-drone repulsion with closing-rate damping.
    for j in range(len(positions)):
        if j == i:
            continue
        d_vec = p_i - positions[j]
        d = np.linalg.norm(d_vec)
        if d < 1e-9 or d >= cp.d_act:
            continue
        n = d_vec / d
        mag = _ramp(d, cp.d_act, cp.d_floor, cp.v_rep_max)
        closing = -np.dot(v_i - velocities[j], n)  # >0 when approaching
        mag += cp.k_damp * max(closing, 0.0)
        u_out += mag * n

    # Obstacle repulsion (sphere primitives; measured to the surface).
    for center, radius in obstacles:
        d_vec = p_i - np.asarray(center)
        d = np.linalg.norm(d_vec) - radius
        if d < 1e-9 or d >= cp.d_act:
            continue
        n = d_vec / np.linalg.norm(d_vec)
        u_out += _ramp(d, cp.d_act, cp.d_floor, cp.v_rep_max) * n

    # Course-boundary repulsion: per-axis push-back inside the margin band.
    lo = cp.bounds[0::2]
    hi = cp.bounds[1::2]
    for ax in range(3):
        if np.isfinite(lo[ax]):
            gap = p_i[ax] - lo[ax]
            if gap < cp.bound_margin:
                u_out[ax] += cp.v_rep_max * (cp.bound_margin - gap) / cp.bound_margin
        if np.isfinite(hi[ax]):
            gap = hi[ax] - p_i[ax]
            if gap < cp.bound_margin:
                u_out[ax] -= cp.v_rep_max * (cp.bound_margin - gap) / cp.bound_margin

    return clip_norm(u_out, cp.v_max)
