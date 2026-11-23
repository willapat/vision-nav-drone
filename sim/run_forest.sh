#!/usr/bin/env bash
# Launch the forest world with PX4 SITL and the x500, in the right order.
#
# PX4's default launch starts the Gazebo GUI and spawns the drone at the same time.
# The forest has hundreds of detailed models, so the GUI is still loading when the
# drone appears and never shows it. This script starts Gazebo first, waits for the
# GUI to finish loading, then starts PX4 in standalone mode to spawn the drone.
#
# It also starts a fresh Micro XRCE-DDS agent: reusing one across PX4 restarts makes
# PX4 segfault on a stale session.
#
# Usage: sim/run_forest.sh [world]        (default world: forest3d)
#   HEADLESS=1   no GUI
#   GUI_WAIT=25  seconds to let the GUI load after it comes up
set -euo pipefail

WORLD=${1:-forest3d}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
GUI_WAIT=${GUI_WAIT:-25}

set +u  # gz_env.sh appends to variables that may be unset
source "$PX4_DIR/build/px4_sitl_default/gz_env.sh"
set -u
export GZ_IP=127.0.0.1

cleanup() {
    echo "Stopping Gazebo and the XRCE-DDS agent..."
    [[ -n "${GZ_PID:-}" ]] && kill "$GZ_PID" 2>/dev/null || true
    [[ -n "${AGENT_PID:-}" ]] && kill "$AGENT_PID" 2>/dev/null || true
    pkill -f "^gz sim .*${WORLD}.sdf" 2>/dev/null || true
    pkill -f "^gz sim -g" 2>/dev/null || true
}
trap cleanup EXIT

if pgrep -f "^gz sim" >/dev/null || pgrep -x px4 >/dev/null; then
    echo "Gazebo or PX4 is already running. Stop it first (pkill -f gz; pkill -x px4)." >&2
    exit 1
fi

pkill -x MicroXRCEAgent 2>/dev/null || true
MicroXRCEAgent udp4 -p 8888 >/tmp/microxrce_agent.log 2>&1 &
AGENT_PID=$!

echo "Starting Gazebo with $WORLD..."
if [[ -n "${HEADLESS:-}" ]]; then
    gz sim -r -s "$PX4_GZ_WORLDS/$WORLD.sdf" &
else
    gz sim -r "$PX4_GZ_WORLDS/$WORLD.sdf" &
fi
GZ_PID=$!

echo "Waiting for the world..."
until gz topic -l 2>/dev/null | grep -q "^/world/$WORLD/clock"; do sleep 1; done
if [[ -z "${HEADLESS:-}" ]]; then
    until gz service -l 2>/dev/null | grep -q '^/gui/follow$'; do sleep 1; done
    echo "GUI is up, letting it load the forest for ${GUI_WAIT}s..."
    sleep "$GUI_WAIT"
fi

echo "Starting PX4..."
export PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:-0,0,0.15}  # spawn on top of the landing pad
cd "$PX4_DIR"
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD" make px4_sitl gz_x500
