#!/usr/bin/env bash
#
# The definition of done from CLAUDE.md, in the form that actually works on
# this machine. `cargo test --workspace` links every test binary at once and
# exhausts the linker here (bevy's debug info is enormous), so the test step
# is split: the small crates link together, and `server` links alone with a
# reduced job count.
#
# Usage:
#   scripts/gate.sh            # everything
#   scripts/gate.sh fmt        # a single step:
#                              #   fmt | clippy | test | test-server
#                              #   test-viewer
#                              #   selftest | clients   (the python side)

set -uo pipefail
cd "$(dirname "$0")/.."

step="${1:-all}"
status=0

run() {
    echo "=== $* ==="
    "$@" || status=1
}

# Same, from a subdirectory: `python3 -m shotgun.selftest` needs clients/python
# to be the working directory, the way a client script gets it for free.
run_in() {
    dir="$1"
    shift
    echo "=== $* (in $dir) ==="
    ( cd "$dir" && "$@" ) || status=1
}

[ "$step" = all ] || [ "$step" = fmt ] && run cargo fmt --check
[ "$step" = all ] || [ "$step" = clippy ] && run cargo clippy --workspace --all-targets -- -D warnings
[ "$step" = all ] || [ "$step" = test ] && run cargo test -p protocol -p movement -p sensors \
    -p transport -p viz -p perception -p map -p map-opendrive
[ "$step" = all ] || [ "$step" = test-server ] && run cargo test -p server -j 2
# `viewer` is excluded from the *testing* requirement (CLAUDE.md: rendering is
# verified by looking at the screen), but it is not excluded from having tests
# run. Anything pure that does end up in there -- camera placement, wheel
# tinting -- was silently never executed until this line existed.
[ "$step" = all ] || [ "$step" = test-viewer ] && run cargo test -p viewer -j 2
[ "$step" = all ] || [ "$step" = selftest ] && run_in clients/python python3 -m shotgun.selftest
[ "$step" = all ] || [ "$step" = clients ] && run python3 scripts/check_clients.py

exit $status
