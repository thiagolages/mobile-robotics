#!/usr/bin/env bash
SCENE_FILE=
# set -euo pipefail

# # Headless with default remote API add-on
# # More flags: -s to run a scene, -q (quiet), -h (no GUI)
# COPPELIA="${COPPELIA_DIR:-/opt/CoppeliaSim}/coppeliaSim.sh"

# # If a scene path is provided, run it; else start empty headless
# SCENE="${1:-}"
# if [[ -n "$SCENE" ]]; then
#   exec xvfb-run -a "$COPPELIA" -h -q -s "$SCENE"
# else
#   exec xvfb-run -a "$COPPELIA" -h -q
# fi

/opt/CoppeliaSim/coppeliaSim.sh -GpreferredSandboxLang=python -GzmqRemoteApi.rpcPort=23000 -GzmqRemoteApi.keepServerAlive=1