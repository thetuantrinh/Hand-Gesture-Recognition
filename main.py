#!/usr/bin/env python3
"""Main entry point for Hand Gesture Recognition & Robotics System.

Run this script directly from the repository root:
    python3 main.py
    python3 main.py --model Uncertainty-Aware/scripts/models/model_1.h5
"""

import os
import sys
import argparse

# Register Uncertainty-Aware/scripts to system path
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(ROOT_DIR, "Uncertainty-Aware", "scripts")

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="FMCW Radar Hand Gesture Recognition & UR3 Robotics Control Center",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="Optional path to custom pre-trained Keras model (.h5 or .keras)",
    )
    return parser.parse_args()


def main() -> None:
    """Initialize GUI application."""
    args = parse_args()

    from PyQt5 import QtWidgets
    from main_control import MainWindow

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(model_path=args.model)
    window.mainWindow.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
