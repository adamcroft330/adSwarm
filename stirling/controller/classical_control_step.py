"""One classical-control tick (tech doc v0.6 §8.1, RL pipeline §2.5).

Python reconstruction of the missing classical_control_step.m — the
four-layer composition:

    u_classic = formation_tracking(target, state)        # every tick
    u_total   = u_classic + k_res * dv                   # RL residual
    v_cmd     = safety_filter(u_total, ...)              # APF, then saturate

Pass dv=None for pure classical control — the Stage 3a benchmark mode
and the runtime fallback (tech doc §10). Returns u_classic alongside
v_cmd so it can be appended to the observation (RL pipeline §2.5).
"""

import numpy as np

from formation_tracking import formation_tracking
from safety_filter import safety_filter


def classical_control_step(i, positions, velocities, p_target, v_target,
                           integ, dv, obstacles, dt, cp):
    """Compute drone i's world velocity setpoint for this tick.

    positions, velocities : (N, 3) all-drone world state
    p_target, v_target    : (3,) this drone's slot target from the manager
    integ                 : (3,) caller-owned integral state
    dv                    : (3,) RL residual, or None for pure classical
    obstacles             : list of (center, radius) spheres

    Returns (v_cmd, integ_new, u_classic).
    """
    u_classic, integ = formation_tracking(
        positions[i], velocities[i], p_target, v_target, integ, dt, cp)

    u_total = u_classic if dv is None else u_classic + cp.k_res * np.asarray(dv)

    v_cmd = safety_filter(u_total, i, positions, velocities, obstacles, cp)
    return v_cmd, integ, u_classic
