"""Interactive MuJoCo viewer for the formation demo.

Runs the same scenario as demo_formation.py in real time with the
native MuJoCo viewer so controller behaviour can be inspected live
(drag to orbit, scroll to zoom; translucent spheres are the live slot
targets). The scenario loops forever; Ctrl-C or close the window to
stop.

On macOS the interactive viewer must run under mjpython:

    mjpython stirling/controller/view_formation.py
"""

import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from classical_control_step import classical_control_step
from default_params import default_params
from demo_formation import CTRL_DT, SUBSTEPS, Scenario, make_model
from drone_model import get_drone_state
from formation_manager import FormationManager, N_DRONES
from quad_autopilot import QuadAutopilot


def main():
    cp, fp, qp = default_params()
    model, data = make_model(qp, fp, seed=0)
    autopilot = QuadAutopilot(qp)
    scen = Scenario()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # (Re)start the scenario from scratch each loop.
            manager = FormationManager(fp)
            integs = np.zeros((N_DRONES, 3))
            kicked = False
            _, fresh = make_model(qp, fp, seed=0)
            data.qpos[:] = fresh.qpos
            data.qvel[:] = 0.0
            t0 = time.monotonic()

            for step in range(int(round(scen.T_END / CTRL_DT))):
                if not viewer.is_running():
                    return
                t = step * CTRL_DT
                manager.set_mode(scen.mode_at(t), t)
                centroid = scen.centroid(t)
                p_targets, v_targets = manager.targets(centroid, t)

                if not kicked and t >= scen.KICK_T:
                    data.qvel[6 * scen.KICK_DRONE:
                              6 * scen.KICK_DRONE + 3] += scen.KICK_V
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
                viewer.sync()

                # Real-time pacing.
                lag = t0 + t + CTRL_DT - time.monotonic()
                if lag > 0:
                    time.sleep(lag)


if __name__ == "__main__":
    main()
