#!/usr/bin/env python3
"""Verify the local MCGGloveTeleop Python environment."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BRIDGES = ("glove_l6_bridge.py", "glove_l20_bridge.py", "glove_o6_bridge.py")
MODULES = ("numpy", "can", "serial", "yaml", "realhand")
SDK_MODULES = ("realhand.hand.l6", "realhand.hand.o6", "realhand.hand.l20")


def compile_bridges() -> list[str]:
    errors: list[str] = []
    for bridge in BRIDGES:
        path = ROOT / bridge
        try:
            compile(path.read_text(encoding="utf-8"), str(path), "exec")
        except Exception as exc:  # noqa: BLE001 - verification should report all failures.
            errors.append(f"{bridge}: {exc}")
    return errors


def import_modules(names: tuple[str, ...]) -> list[str]:
    errors: list[str] = []
    for name in names:
        try:
            importlib.import_module(name)
        except Exception as exc:  # noqa: BLE001 - verification should report all failures.
            errors.append(f"{name}: {exc}")
    return errors


def main() -> int:
    errors = []
    errors.extend(compile_bridges())
    errors.extend(import_modules(MODULES))
    errors.extend(import_modules(SDK_MODULES))

    if errors:
        print("Environment check failed:")
        for error in errors:
            print(f"  - {error}")
        return 1

    print("Environment check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
