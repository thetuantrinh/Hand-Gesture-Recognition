"""Widget construction for the main window, split by region.

Each module contributes a mixin with a ``build_*`` and a ``retranslate_*``
method; :class:`~radar_hgr.ui.layout.main_window.MainWindowLayout` composes
them in a fixed order.
"""

from .main_window import PANELS, MainWindowLayout

__all__ = ["PANELS", "MainWindowLayout"]
