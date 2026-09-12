"""Allows the application to be started with ``python -m radar_hgr``."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
