"""Data-file lookup for the vendored Universal Robots interface.

Upstream used ``pkg_resources.resource_filename``, which is deprecated and no
longer ships with current setuptools. The two XML configuration files live
alongside these modules, so they are resolved from ``__file__`` instead.
"""

from pathlib import Path

SDK_DIR = Path(__file__).resolve().parent

#: Where the UR event and data logs are written when no path is supplied.
DEFAULT_LOG_DIR = SDK_DIR.parents[2] / "ur_log"


def resource_filename(_module_name: str, filename: str) -> str:
    """Absolute path of a data file bundled with this package.

    The first argument is accepted and ignored, so call sites read the same as
    the ``pkg_resources`` function they replace.
    """
    return str(SDK_DIR / filename)
