"""Lookup of UI assets bundled with the package."""

from pathlib import Path

ASSETS_DIR: Path = Path(__file__).resolve().parent / "assets"
"""Directory holding icons, logos and pixmaps."""

DESIGNER_DIR: Path = Path(__file__).resolve().parent / "designer"
"""Directory holding the Qt Designer source for the main window."""


def asset(relative_path: str) -> str:
    """Absolute path of a bundled asset, as a string for the Qt constructors.

    Parameters
    ----------
    relative_path:
        Path within :data:`ASSETS_DIR`, e.g. ``"icon/up.png"``.
    """
    return str(ASSETS_DIR / relative_path)
