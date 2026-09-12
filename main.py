#!/usr/bin/env python3
"""Repository-root launcher for the real-time gesture recognition application.

Provided for convenience so the application can be started from a fresh clone
without installing it:

    python3 main.py
    python3 main.py --model Uncertainty-Aware/models/model_1.h5

Installing the package instead (``pip install -e Uncertainty-Aware``) provides
the ``radar-hgr`` console script, which is the supported entry point.
"""

import sys
from pathlib import Path

PACKAGE_SRC = Path(__file__).resolve().parent / "Uncertainty-Aware" / "src"

if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from radar_hgr.cli import main  # noqa: E402 - requires the path set up above

if __name__ == "__main__":
    sys.exit(main())
