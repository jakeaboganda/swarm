#!/usr/bin/env python3
"""Byte-compile and import every client in clients/python/.

Importing is the half that earns its keep: it resolves `from shotgun import
lane_plan`, so a helper that was renamed or deleted fails here instead of in
front of a running sim. Clients guard `main()` behind `__name__ ==
"__main__"`, so nothing connects to anything.

`websockets` is optional -- a client that exits at import because it is
missing counts as skipped, not failed. Any other failure is a failure.

Usage: python3 scripts/check_clients.py
"""

import importlib
import importlib.util
import py_compile
import sys
import traceback
from pathlib import Path

CLIENTS = Path(__file__).resolve().parent.parent / "clients" / "python"
SKIP_EXIT = 2  # what a client exits with when websockets is absent


def main():
    have_websockets = importlib.util.find_spec("websockets") is not None
    sys.path.insert(0, str(CLIENTS))

    failures, skipped, checked = [], [], []
    for path in sorted(CLIENTS.glob("*.py")):
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as e:
            failures.append((path.name, str(e)))
            continue
        try:
            importlib.import_module(path.stem)
        except SystemExit as e:
            if e.code == SKIP_EXIT and not have_websockets:
                skipped.append(path.name)
            else:
                failures.append((path.name, f"exited {e.code} on import"))
            continue
        except BaseException:
            failures.append((path.name, traceback.format_exc()))
            continue
        checked.append(path.name)

    for name, why in failures:
        print(f"FAIL {name}: {why}")
    if skipped:
        print(f"skipped (no websockets installed): {', '.join(skipped)}")
    print(f"{len(checked)}/{len(checked) + len(failures)} clients compile and import")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
