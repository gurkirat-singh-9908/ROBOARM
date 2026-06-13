#!/bin/bash
# Start the RoboArm web interface behind an ngrok tunnel.
# Recreates the roboenv venv if missing, then runs the Flask app on :8080.
set -euo pipefail

ROOT="$HOME/ROBOARM"
REQS="$ROOT/ManualPhase1/WebBasedIKV2/website_dev/requirements.txt"
APP="$ROOT/ManualPhase1/WebBasedIKV2/website_dev/app.py"
NGROK_URL="https://flying-scorpion-neat.ngrok-free.app"

if [ ! -d "$ROOT/roboenv" ]; then
    echo "roboenv not found, recreating..."
    python3 -m venv "$ROOT/roboenv"
    "$ROOT/roboenv/bin/pip" install -r "$REQS"
fi

# shellcheck disable=SC1091
source "$ROOT/roboenv/bin/activate"

NGROK_PID=""
if command -v ngrok >/dev/null 2>&1; then
    ngrok http 8080 --url "$NGROK_URL" &
    NGROK_PID=$!
    # Kill the tunnel when this script exits (Ctrl-C, app crash, etc.)
    trap '[ -n "$NGROK_PID" ] && kill "$NGROK_PID" 2>/dev/null' EXIT
else
    echo "WARNING: ngrok not found on PATH — starting web app without a public tunnel."
fi

python3 "$APP"
