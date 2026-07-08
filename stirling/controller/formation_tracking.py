"""Nominal formation-tracking law (tech doc v0.6 §8.2).

Python reconstruction of the missing formation_tracking.m:
    u = kff * v_target + kp * (p_target - p) + ki * integ(e)
P + feedforward + light-I with conditional anti-windup. Pure function of
(state, target, integrator) -> (velocity setpoint, new integrator); maps
directly to C.
"""

import numpy as np


def clip_norm(v, limit):
    n = np.linalg.norm(v)
    if n > limit:
        return v * (limit / n)
    return v


def formation_tracking(p, v, p_target, v_target, integ, dt, cp):
    """One tick of the nominal law for a single drone.

    p, v            : drone world position / velocity (3,)
    p_target        : world slot target from the formation manager (3,)
    v_target        : world target velocity (feedforward) (3,)
    integ           : integral state (3,), caller-owned
    dt              : controller period [s]
    cp              : ControllerParams

    Returns (u, integ_new): unsaturated-then-clipped world velocity
    setpoint and updated integrator.
    """
    e = p_target - p
    u_raw = cp.kff * v_target + cp.kp * e + cp.ki * integ
    u = clip_norm(u_raw, cp.v_max)

    # Conditional anti-windup: only integrate while the command is not
    # saturated, and clamp the stored integral per axis.
    if np.linalg.norm(u_raw) < cp.v_max:
        integ = np.clip(integ + e * dt, -cp.i_limit, cp.i_limit)
    return u, integ
