#!/usr/bin/env bash
#
# Tear down anything left running from the stack: the headless simulation
# (which binds the agent + viz ports), the reference viewer, and the demo
# agents. Safe to run anytime — it's a no-op if nothing is up.
#
# Usage:
#   scripts/kill.sh

set -uo pipefail
cd "$(dirname "$0")/.."

# The server binds these; freeing them kills it. Viewer and agents only
# connect out, so they're matched by name.
fuser -k 4000/tcp 4001/tcp 2>/dev/null
pkill -f 'target/debug/(server|viewer)' 2>/dev/null
pkill -f 'clients/python/' 2>/dev/null

sleep 0.5

# Ports are the source of truth: if they're free, the sim is truly down.
if ss -ltn 2>/dev/null | grep -qE ':4000 |:4001 '; then
    echo "still bound:"
    ss -ltnp 2>/dev/null | grep -E ':4000 |:4001 '
    exit 1
fi
echo "all down — ports 4000/4001 free."
