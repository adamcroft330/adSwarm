#!/usr/bin/env bash
# Stage 3a classical benchmark (stage3_plan.md task f).
#
#   bash stirling/tests/run_stage3a_bench.sh
#
# Records the FORMATION floor the Stage 3b residual must beat. Pure classical
# control (k_res=0) — no policy, no GPU, no Modal. Headless, ~10 s.
#
# This is a measurement, not a gate: it always exits 0 unless the build or the
# run itself fails. The numbers are recorded in stirling/docs/progress_log.md;
# re-run it after any change to the controller, the reward, or the dynamics and
# compare, because the 3a floor is only meaningful against a 3b residual
# measured on the same env.
set -uo pipefail
cd "$(dirname "$0")/../.."

OUT="${TMPDIR:-/tmp}/stirling_stage3a_bench"
mkdir -p "$OUT"
"${CC:-clang}" -O2 -I ocean/drone -I src stirling/tests/bench_stage3a.c \
    -lm -o "$OUT/bench" || exit 2
"$OUT/bench"
