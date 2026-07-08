"""MuJoCo model of 4 generic 250-class quadrotors.

Builds the MJCF programmatically from QuadParams so the physical
constants live in exactly one place (default_params.py). Each drone is a
free body with explicit inertial properties, one collision sphere sized
to the prop-guard envelope, and 4 site-transmission actuators whose gear
couples body-z force with the rotor reaction torque (yaw). Roll/pitch
moments arise naturally from the thrust application points.

Translucent mocap spheres visualise the live slot targets.
"""

import numpy as np

DRONE_RGBA = [
    "0.90 0.25 0.20 1",  # slot 0 red
    "0.20 0.55 0.95 1",  # slot 1 blue
    "0.25 0.75 0.30 1",  # slot 2 green
    "0.95 0.75 0.15 1",  # slot 3 yellow
]


def _drone_body(i, qp, spawn):
    a = qp.arm / np.sqrt(2.0)
    motors = [(a, a), (a, -a), (-a, -a), (-a, a)]
    rgba = DRONE_RGBA[i]
    arms = "\n".join(
        f'      <geom type="capsule" fromto="0 0 0 {x:.4f} {y:.4f} 0" size="0.008"'
        f' rgba="0.15 0.15 0.15 1" mass="0" group="1" contype="0" conaffinity="0"/>'
        for x, y in motors)
    rotors = "\n".join(
        f'      <geom type="cylinder" pos="{x:.4f} {y:.4f} 0.012" size="0.062 0.004"'
        f' rgba="0.3 0.3 0.35 0.6" mass="0" group="1" contype="0" conaffinity="0"/>'
        for x, y in motors)
    sites = "\n".join(
        f'      <site name="m{i}_{k}" pos="{x:.4f} {y:.4f} 0.012" size="0.005"/>'
        for k, (x, y) in enumerate(motors))
    return f"""
    <body name="drone{i}" pos="{spawn[0]:.3f} {spawn[1]:.3f} {spawn[2]:.3f}">
      <freejoint name="drone{i}_free"/>
      <inertial pos="0 0 0" mass="{qp.mass}"
        diaginertia="{qp.inertia[0]} {qp.inertia[1]} {qp.inertia[2]}"/>
      <geom name="guard{i}" type="sphere" size="{qp.guard_radius}"
        rgba="{rgba.rsplit(' ', 1)[0]} 0.10" group="2"/>
      <geom type="box" size="0.045 0.045 0.018" rgba="{rgba}" mass="0"
        group="1" contype="0" conaffinity="0"/>
{arms}
{rotors}
{sites}
    </body>"""


def _target_body(i):
    rgba = DRONE_RGBA[i].rsplit(" ", 1)[0] + " 0.25"
    return f"""
    <body name="target{i}" mocap="true" pos="0 0 -1">
      <geom type="sphere" size="0.06" rgba="{rgba}" contype="0" conaffinity="0" group="1"/>
    </body>"""


def _actuators(n, qp):
    out = []
    for i in range(n):
        for k in range(4):
            spin = 1.0 if k % 2 == 0 else -1.0
            out.append(
                f'    <general name="thrust{i}_{k}" site="m{i}_{k}"'
                f' gear="0 0 1 0 0 {spin * qp.km_over_kf:.4f}"'
                f' ctrlrange="0 {qp.thrust_max}"/>')
    return "\n".join(out)


def build_mjcf(qp, spawns, timestep=0.002):
    bodies = "\n".join(_drone_body(i, qp, s) for i, s in enumerate(spawns))
    targets = "\n".join(_target_body(i) for i in range(len(spawns)))
    return f"""
<mujoco model="stirling_formation">
  <option timestep="{timestep}" density="1.225" viscosity="1.8e-5"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
    <quality shadowsize="4096"/>
    <map znear="0.05"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.45 0.65 0.9" rgb2="0.9 0.95 1.0"
      width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.22 0.28 0.22"
      rgb2="0.28 0.34 0.28" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="10 10" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="60 20 0.1" material="grid"/>
{bodies}
{targets}
  </worldbody>
  <actuator>
{_actuators(len(spawns), qp)}
  </actuator>
</mujoco>"""


def get_drone_state(data, i):
    """(p world, v world, R body-to-world, omega body) for drone i."""
    q0 = 7 * i
    v0 = 6 * i
    p = data.qpos[q0:q0 + 3].copy()
    v = data.qvel[v0:v0 + 3].copy()
    omega = data.qvel[v0 + 3:v0 + 6].copy()  # free joint: body frame
    quat = data.qpos[q0 + 3:q0 + 7]
    w, x, y, z = quat
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return p, v, R, omega
