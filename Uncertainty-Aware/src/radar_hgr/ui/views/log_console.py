"""The timestamped log console in the right-hand pane."""

from typing import Any

from PyQt5.QtCore import QDateTime

TIMESTAMP_FORMAT = "yyyy-MM-dd hh:mm:ss"


class LogConsole:
    """Appends timestamped lines to a read-only ``QTextEdit``, following the tail.

    Parameters
    ----------
    text_edit:
        The read-only ``QTextEdit`` to append to.
    """

    def __init__(self, text_edit: Any) -> None:
        self.text_edit = text_edit

    def append(self, message: str) -> None:
        """Append one timestamped line and scroll to it."""
        timestamp = QDateTime.currentDateTime().toString(TIMESTAMP_FORMAT)
        self.text_edit.append(f"[{timestamp}] {message}")

        scrollbar = self.text_edit.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
