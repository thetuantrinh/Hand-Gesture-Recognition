"""Command line entry point."""

import argparse
import sys
from typing import Optional

from . import __version__
from .logging_config import configure_logging


def build_parser() -> argparse.ArgumentParser:
    """Construct the argument parser."""
    parser = argparse.ArgumentParser(
        prog="radar-hgr",
        description=(
            "Uncertainty-aware FMCW radar hand gesture recognition for "
            "Universal Robots UR3 control."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--model",
        metavar="PATH",
        default=None,
        help="Keras checkpoint (.h5 or .keras) to load instead of the default",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="log at DEBUG level, including the radar protocol exchange",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    """Parse arguments, configure logging, and run the GUI.

    Returns
    -------
    int
        Process exit code.
    """
    args = build_parser().parse_args(argv)
    configure_logging(verbose=args.verbose)

    # Imported after logging is configured, and only once the arguments are
    # known to be valid, so `--help` does not pay Qt's import cost.
    from .app.application import run

    return run(model_path=args.model)


if __name__ == "__main__":
    sys.exit(main())
