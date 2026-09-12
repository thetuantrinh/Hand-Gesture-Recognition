"""Qt application bootstrap."""

import logging
import sys
from typing import Optional

from PyQt5 import QtWidgets

from .main_window import MainWindow

logger = logging.getLogger(__name__)


def run(model_path: Optional[str] = None, argv: Optional[list[str]] = None) -> int:
    """Start the GUI and block until the operator closes it.

    Parameters
    ----------
    model_path:
        Classifier checkpoint to load at startup.
    argv:
        Arguments handed to ``QApplication``. Defaults to ``sys.argv``.

    Returns
    -------
    int
        The Qt exit code, suitable for :func:`sys.exit`.
    """
    app = QtWidgets.QApplication(argv if argv is not None else sys.argv)

    window = MainWindow(model_path=model_path)
    window.show()
    logger.info("Application ready")

    return app.exec_()
