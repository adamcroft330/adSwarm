#!/usr/bin/env bash
# macOS local visual eval setup for the PufferLib drone environment.
#
# This builds the CPU backend used by:
#   puffer eval drone --slowly --load-model-path <checkpoint.bin>
set -euo pipefail

cd "$(dirname "$0")/../.."   # repo root

if [ "$(uname -s)" != "Darwin" ]; then
    echo "ERROR: macos_eval_setup.sh is intended for macOS only."
    exit 1
fi

if ! command -v brew >/dev/null 2>&1; then
    echo "ERROR: Homebrew is required. Install it from https://brew.sh/ and rerun this script."
    exit 1
fi

BASE_PYTHON="${PYTHON:-python3}"
if ! command -v "$BASE_PYTHON" >/dev/null 2>&1; then
    echo "ERROR: $BASE_PYTHON not found. Install Python 3 and rerun this script."
    exit 1
fi

if [ -n "${VIRTUAL_ENV:-}" ]; then
    PYTHON="$BASE_PYTHON"
    echo "== using active venv: $VIRTUAL_ENV =="
else
    VENV_DIR="${VENV_DIR:-.venv-macos-eval}"
    if [ ! -x "$VENV_DIR/bin/python" ]; then
        "$BASE_PYTHON" -m venv "$VENV_DIR"
    fi
    PYTHON="$VENV_DIR/bin/python"
fi

echo "== repo: $(pwd)  branch: $(git rev-parse --abbrev-ref HEAD) =="
echo "== python: $("$PYTHON" --version) =="

for pkg in libomp raylib ccache; do
    if brew --prefix "$pkg" >/dev/null 2>&1; then
        echo "brew: $pkg present at $(brew --prefix "$pkg")"
    else
        echo "brew: installing $pkg"
        brew install "$pkg"
    fi
done

LIBOMP_PREFIX="$(brew --prefix libomp)"
export LIBOMP_PREFIX
export CPPFLAGS="${CPPFLAGS:-} -I$LIBOMP_PREFIX/include"
export LDFLAGS="${LDFLAGS:-} -L$LIBOMP_PREFIX/lib"

"$PYTHON" -m pip install --upgrade pip wheel
"$PYTHON" -m pip install -e .

PYTHON="$PYTHON" bash build.sh drone --cpu
"$PYTHON" -c "import pufferlib, pufferlib._C as C; print(f'pufferlib _C import OK: env={C.env_name} gpu={C.gpu}')"

echo
echo "== macOS eval setup complete =="
if [ -z "${VIRTUAL_ENV:-}" ]; then
    echo "Use this environment for eval commands: source $VENV_DIR/bin/activate"
fi
echo "Checkpoint eval: bash stirling/scripts/eval_stage1_macos.sh <checkpoint.bin>"
echo "Random/render smoke: puffer eval drone --slowly"
echo "Native packaged-weights smoke: bash stirling/scripts/eval_pretrained_drone_macos.sh"
