"""Qt presentation layer.

:mod:`layout` builds the widget tree, :mod:`views` adapts application state for
display, and :mod:`resources` locates the bundled icons and pixmaps.
"""

from .layout import MainWindowLayout
from .resources import asset

__all__ = ["MainWindowLayout", "asset"]
