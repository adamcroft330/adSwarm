"""Default parameters for the classical formation controller.

Python reconstruction of the missing MATLAB draft's default_params.m
(tech doc v0.6 §8.1). All gains, limits, and safety margins live here —
tune here first (§8.5). This file is the import blueprint for the
eventual C port into ocean/drone (velocity_controller.h).

Drone-physical values are GENERIC 250-class quad numbers (NFR-16/17/18:
TOW <= 1.5 kg, <= 250 mm envelope, 4S), not a selected platform — Stage 2
recalibration replaces them once hardware is picked.
"""

from dataclasses import dataclass, field

import numpy as np

GRAVITY = 9.81


@dataclass
class ControllerParams:
    # --- Nominal tracking law (tech doc §8.2) --------------------------------
    # u = kff*v_target + kp*(p_target - p) + ki*integ(e)
    kff: float = 1.0     # feedforward on target velocity (prevents lag)
    kp: float = 2.0      # closed-loop time constant ~0.5 s
    ki: float = 0.3      # light-I: kills steady-state offset
    i_limit: float = 0.5  # per-axis integrator clamp [m*s]
    v_max: float = 3.0   # velocity-setpoint saturation [m/s]

    # --- APF safety filter (tech doc §8.3) -----------------------------------
    # d_act sits just below the tightest steady-state formation spacing
    # (compressed: 0.72 m, stack: 0.75 m) so the APF never fights a held
    # formation; the closing-rate damping term still brakes fast approaches.
    d_floor: float = 0.40   # hard separation floor [m] (FR-15 TBC)
    d_act: float = 0.70     # inter-drone repulsion activation distance [m]
    v_rep_max: float = 2.0  # max repulsion speed at the floor [m/s]
    k_damp: float = 0.8     # closing-rate damping gain on inter-drone term
    bound_margin: float = 0.5   # course-boundary repulsion band [m]
    # course bounds [xmin xmax ymin ymax zmin zmax]; +/-inf disables a face
    bounds: np.ndarray = field(default_factory=lambda: np.array(
        [-np.inf, np.inf, -np.inf, np.inf, 0.4, 6.0]))

    # --- Residual interface (RL pipeline §2.5) -------------------------------
    k_res: float = 1.0   # residual scale; dv=None -> pure classical


@dataclass
class FormationParams:
    box_side: float = 1.2        # home box side length [m]
    line_spacing: float = 1.0    # single-file spacing [m]
    stack_spacing: float = 0.75  # vertical column spacing [m] (> d_act)
    compressed_scale: float = 0.6  # compressed square = box * scale (sep 0.72 > d_act)
    diamond_long: float = 1.0    # lead/trail offset [m]
    diamond_wide: float = 0.7    # side-wing offset [m]
    blend_time: float = 1.0      # linear mode-transition blend [s]


@dataclass
class QuadParams:
    """Generic 250-class quadrotor + PX4-like velocity-mode inner loop."""
    mass: float = 0.9        # [kg], inside NFR-16
    arm: float = 0.125       # centre-to-motor [m] -> 250 mm motor diagonal
    inertia: np.ndarray = field(default_factory=lambda: np.array(
        [0.006, 0.006, 0.010]))  # diag [kg m^2]
    thrust_max: float = 8.0  # per motor [N] -> TWR ~3.6
    km_over_kf: float = 0.016  # rotor reaction-torque / thrust ratio [m]
    guard_radius: float = 0.125  # collision sphere incl. prop guards [m]

    # velocity inner loop: a_cmd = kv*(v_sp - v) + g
    kv: float = 5.0
    tilt_max: float = np.deg2rad(35.0)
    # attitude PD (torque = I @ (kr*(-e_R) + kw*(-omega)))
    kr_rp: float = 200.0
    kw_rp: float = 25.0
    kr_yaw: float = 30.0
    kw_yaw: float = 8.0


def default_params():
    return ControllerParams(), FormationParams(), QuadParams()
