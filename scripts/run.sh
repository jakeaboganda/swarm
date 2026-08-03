#!/usr/bin/env bash
#
# Launch the full stack: the headless simulation, the reference viewer, and
# the demo agents that drive the cars. Ctrl-C tears everything down.
#
# Usage:
#   scripts/run.sh [scenario.json] [agent-client.py]
#
# Defaults to scenario.json and the patrol demo. The viewer reconnects on
# its own, so ordering doesn't matter; agents wait for the sim to listen.

set -uo pipefail
cd "$(dirname "$0")/.."

SCENARIO="${1:-scenario.json}"
AGENTS="${2:-clients/python/patrol_demo.py}"

pids=()
cleanup() {
    trap - EXIT INT TERM
    kill "${pids[@]}" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_port() {
    local port="$1" tries=40
    until ss -ltn 2>/dev/null | grep -q ":$port "; do
        tries=$((tries - 1))
        [ "$tries" -le 0 ] && { echo "timed out waiting for port $port"; return 1; }
        sleep 0.25
    done
}

# Free the ports up front so a straggler from a previous run (a server that
# ignored Ctrl-C, say) can't block this one. See scripts/kill.sh.
if ss -ltn 2>/dev/null | grep -qE ':4000 |:4001 '; then
    echo "freeing ports held by a previous run..."
    fuser -k 4000/tcp 4001/tcp 2>/dev/null
    sleep 0.5
fi

echo "building server + viewer..."
cargo build --bin server --bin viewer

echo "starting headless simulation ($SCENARIO)..."
./target/debug/server "$SCENARIO" &
pids+=($!)

echo "starting viewer..."
./target/debug/viewer &
pids+=($!)

# Drive the cars once the agent port is up.
wait_for_port 4000 || exit 1
echo "starting agents ($AGENTS)..."
python3 "$AGENTS" &
pids+=($!)

echo "running — press Ctrl-C to stop."
wait
