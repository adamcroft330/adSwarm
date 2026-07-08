"""Velocity-mode inner loop for a generic quadrotor (RL pipeline §2.4).

This is the autopilot layer BELOW the classical controller: it consumes
the world-frame velocity setpoint (what PX4 offboard / ArduPilot guided
mode consumes on hardware) and produces per-motor thrusts. It exists so
the controller can be validated against full rigid-body dynamics in
MuJoCo rather than the MATLAB draft's ideal velocity dynamics.

Cascade: velocity P (+gravity ff) -> desired thrust vector -> desired
attitude (tilt-limited, yaw held) -> attitude PD -> motor mixing.
"""

import numpy as np

from default_params import GRAVITY


def mixer_matrix(qp):
    """A @ motor_thrusts = [T, tau_x, tau_y, tau_z]; returns inv(A).

    X config, arms at 45 deg: motor order front-left, front-right,
    rear-right, rear-left; spin +,-,+,- (CCW positive reaction torque).
    """
    a = qp.arm / np.sqrt(2.0)
    r = np.array([[ a,  a], [ a, -a], [-a, -a], [-a,  a]])  # (x, y) per motor
    spin = np.array([1.0, -1.0, 1.0, -1.0])
    A = np.stack([
        np.ones(4),            # total thrust
        r[:, 1],               # tau_x = sum y_i * T_i
        -r[:, 0],              # tau_y = sum -x_i * T_i
        spin * qp.km_over_kf,  # tau_z from rotor reaction
    ])
    return np.linalg.inv(A)


def _vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


class QuadAutopilot:
    def __init__(self, qp):
        self.qp = qp
        self.mix_inv = mixer_matrix(qp)

    def motor_thrusts(self, R, v, omega_body, v_sp, yaw_sp=0.0):
        """One inner-loop tick.

        R          : (3,3) body-to-world rotation
        v          : (3,) world velocity
        omega_body : (3,) body-frame angular velocity
        v_sp       : (3,) world velocity setpoint from the classical stack

        Returns (4,) motor thrusts [N], clipped to [0, thrust_max].
        """
        qp = self.qp

        # Velocity loop -> desired specific force (world).
        a_cmd = qp.kv * (v_sp - v)
        f_des = qp.mass * (a_cmd + np.array([0.0, 0.0, GRAVITY]))

        # Tilt limit: cap the horizontal component relative to vertical.
        fz = max(f_des[2], 0.3 * qp.mass * GRAVITY)
        f_h = f_des[:2]
        h_max = fz * np.tan(qp.tilt_max)
        h_norm = np.linalg.norm(f_h)
        if h_norm > h_max:
            f_h = f_h * (h_max / h_norm)
        f_des = np.array([f_h[0], f_h[1], fz])

        # Desired attitude from thrust direction + yaw setpoint.
        zb = f_des / np.linalg.norm(f_des)
        xc = np.array([np.cos(yaw_sp), np.sin(yaw_sp), 0.0])
        yb = np.cross(zb, xc)
        yb /= np.linalg.norm(yb)
        xb = np.cross(yb, zb)
        R_des = np.column_stack([xb, yb, zb])

        # Attitude PD (SO(3) error), torque in body frame.
        e_R = 0.5 * _vee(R_des.T @ R - R.T @ R_des)
        kr = np.array([qp.kr_rp, qp.kr_rp, qp.kr_yaw])
        kw = np.array([qp.kw_rp, qp.kw_rp, qp.kw_yaw])
        tau = qp.inertia * (-kr * e_R - kw * omega_body)

        # Thrust magnitude along the actual body z-axis.
        thrust = float(f_des @ R[:, 2])
        thrust = np.clip(thrust, 0.0, 4.0 * qp.thrust_max)

        t = self.mix_inv @ np.array([thrust, tau[0], tau[1], tau[2]])
        return np.clip(t, 0.0, qp.thrust_max)
