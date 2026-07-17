#!/usr/bin/env bash
# Stage 3 velocity-controller test suite (tasks a + b + c).
#
#   bash stirling/tests/run_velocity_tests.sh
#
# Headless: no raylib / vecenv / OpenMP / GPU. Exits non-zero on gate failure.
#
# Hard gates:
#   - native motor path (control_mode=0) still runs
#   - pure classical velocity control hovers and settles
#   - feedforward tracks a moving target without lag
#   - APF repulsion reaches its analytic equilibrium
#   - NFR-36: formation reform < 2.0 s, min separation >= 0.40 m, matching the
#     Python/MuJoCo reference (1.29 s / 0.49 m / 0.76 m worst error)
#   - FORMATION task: slot geometry matches the reference for all 5 modes,
#     mode-transition blend is exact, v_target equals d(p_target)/dt across
#     cruise/turn/blend (the KFF invariant), and the in-env task holds cruise
#     tracking + the separation floor
#
# Also printed (not gates):
#   - APF envelope: where distance-based APF stops holding and CBF-QP (§8.3)
#     would be required.
#   - Motor-lag witness: the same law across k_mot values, showing why the
#     cascade is tuned as it is and what raising BASE_K_MOT would cost.
set -uo pipefail
cd "$(dirname "$0")/../.."

OUT="${TMPDIR:-/tmp}/stirling_vc_tests"
mkdir -p "$OUT"
"${CC:-clang}" -O2 -I ocean/drone -I src stirling/tests/test_velocity_wrapper.c \
    -lm -o "$OUT/tvw" || exit 2
"$OUT/tvw"
rc=$?

echo
if [ "$rc" -ne 0 ]; then
    echo "GATE FAILED — see above."
    exit 1
fi
echo "All gates passed (incl. NFR-36 vs the Python/MuJoCo reference)."
exit 0
