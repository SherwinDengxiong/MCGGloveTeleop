#!/usr/bin/env python3
"""L20 default entrypoint for the G7s UDP glove bridge."""

from __future__ import annotations

import sys

from glove_l6_bridge import main


def with_l20_default(argv: list[str]) -> list[str]:
    if "--hand-model" in argv or any(item.startswith("--hand-model=") for item in argv):
        return argv
    return ["--hand-model", "l20", *argv]


if __name__ == "__main__":
    sys.exit(main(with_l20_default(sys.argv[1:])))
