"""Uncertainty-aware FMCW radar hand gesture recognition for UR3 control.

The package is layered, with dependencies pointing strictly downwards:

``config``
    Frozen dataclasses describing the hardware, the network and the feature
    geometry. Depends on nothing.
``dsp``, ``radar``, ``inference``, ``robot``
    The four domains: signal processing, capture-card I/O, classification, and
    manipulator control. Each depends only on ``config``.
``workers``
    Qt threads that keep acquisition, telemetry and inference off the GUI
    thread.
``ui``
    Widget construction and presentation adapters. No application logic.
``app``
    Wiring: sessions that own hardware lifecycles, and the window that
    connects widgets to them.

The supported entry point is :func:`radar_hgr.app.application.run`, reached via
``python -m radar_hgr`` or the ``radar-hgr`` console script.
"""

__version__ = "1.0.0"

__all__ = ["__version__"]
