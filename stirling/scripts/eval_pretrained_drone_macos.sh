#!/usr/bin/env bash
# Run the packaged native pretrained drone policy on macOS.
#
# This uses resources/drone/drone_weights.bin through the standalone C drone
# renderer. It is not a PyTorch checkpoint and should not be passed to
# puffer eval --slowly --load-model-path.
set -euo pipefail

cd "$(dirname "$0")/../.."   # repo root

if [ "$(uname -s)" != "Darwin" ]; then
    echo "ERROR: eval_pretrained_drone_macos.sh is intended for macOS only."
    exit 1
fi

WEIGHTS="resources/drone/drone_weights.bin"
if [ ! -f "$WEIGHTS" ]; then
    echo "ERROR: packaged drone weights not found: $WEIGHTS"
    exit 1
fi

if ! command -v brew >/dev/null 2>&1; then
    echo "ERROR: Homebrew is required. Install it from https://brew.sh/ and rerun this script."
    exit 1
fi

for pkg in libomp raylib ccache; do
    if ! brew --prefix "$pkg" >/dev/null 2>&1; then
        echo "ERROR: missing Homebrew package: $pkg"
        echo "Run: bash stirling/scripts/macos_eval_setup.sh"
        exit 1
    fi
done

bash build.sh drone --fast
exec ./drone
