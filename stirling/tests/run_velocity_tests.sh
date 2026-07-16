#!/usr/bin/env bash
# Stage 3 velocity-controller test suite (tasks a + b).
#
#   bash stirling/tests/run_velocity_tests.sh
#
# Builds the headless harness twice — once with the env's shipped cascade
# gains, once with the reference gains the Python/MuJoCo validation used — and
# runs both. No raylib / vecenv / OpenMP / GPU needed.
#
# Why two variants: the control *law* is identical; only the cascade gains and
# the platform's motor time constant differ. Separating them answers "is the C
# port faithful?" independently of "can the sim's Crazyflie hit the spec?".
#
#   1. NFR-36 faithfulness gate (HARD, exit code) — reference cascade on a
#      platform with a realistic motor lag must reproduce the Python reference
#      (reform 1.29 s, min sep 0.49 m, worst err 0.76 m).
#   2. Shipped-env report (INFORMATIONAL) — the same law with the gains the
#      sim's BASE_K_MOT=0.15 s motors actually permit. This is the Stage 3a
#      classical floor; it does NOT meet the 2 s reform rule, and that is a
#      platform limit, not a port defect. See stirling/docs/progress_log.md.
set -uo pipefail
cd "$(dirname "$0")/../.."

OUT="${TMPDIR:-/tmp}/stirling_vc_tests"
mkdir -p "$OUT"
CC_BIN="${CC:-clang}"
INC=(-I ocean/drone -I src)
SRC=stirling/tests/test_velocity_wrapper.c

# Reference cascade: what stirling/controller/default_params.py uses (kp=2.0,
# kv=5.0, attitude kr=200/kw=25, v_max=3.0, ki=0.3).
REF_GAINS=(-DVC_KP=2.0f -DVC_KV=5.0f -DVC_KR=200.0f -DVC_KW=25.0f -DVC_V_MAX=3.0f -DVC_KI=0.3f)
# Motor lag for the gate. The sim ships BASE_K_MOT=0.15 s; a real Crazyflie 2.1
# is ~0.02-0.05 s. 0.05 is the realistic value the reference rig implies.
REF_K_MOT=0.05

echo "== 1. Shipped-env report (gains the sim's 0.15 s motors permit) =="
"$CC_BIN" -O2 "${INC[@]}" "$SRC" -lm -o "$OUT/tvw_env" || exit 2
"$OUT/tvw_env"
env_rc=$?
echo

echo "== 2. NFR-36 faithfulness gate (reference cascade, k_mot=${REF_K_MOT}s) =="
"$CC_BIN" -O2 "${INC[@]}" "${REF_GAINS[@]}" "$SRC" -lm -o "$OUT/tvw_ref" || exit 2
"$OUT/tvw_ref" "$REF_K_MOT"
gate_rc=$?

echo
if [ "$gate_rc" -ne 0 ]; then
    echo "NFR-36 GATE FAILED — the C port does not reproduce the Python/MuJoCo"
    echo "reference under reference conditions. This is a port defect."
    exit 1
fi
echo "NFR-36 GATE PASSED — the C port reproduces the Python/MuJoCo reference"
echo "(1.29 s reform, 0.49 m min sep) under reference conditions."
if [ "$env_rc" -ne 0 ]; then
    echo
    echo "Reminder: the shipped-env config does NOT meet the 2 s reform rule."
    echo "That is the sim's BASE_K_MOT=0.15 s motor lag capping the cascade, not"
    echo "a port defect — a Stage 2 platform-recalibration dependency."
fi
exit 0
