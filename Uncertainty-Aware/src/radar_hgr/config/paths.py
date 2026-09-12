"""Resolution of on-disk locations for models and vendored configuration files.

Paths are derived from the installed package location so the application runs
identically from a source checkout and from an installed wheel. Both
directories can be redirected with environment variables, which is how the
deployment images point at an external model store.
"""

import os
from pathlib import Path
from typing import Optional

PACKAGE_DIR: Path = Path(__file__).resolve().parents[1]
"""Root of the importable ``radar_hgr`` package."""

PROJECT_DIR: Path = PACKAGE_DIR.parents[1]
"""Source checkout root (the directory holding ``src/`` and ``models/``)."""

MODELS_ENV_VAR = "RADAR_HGR_MODELS_DIR"
DEFAULT_MODEL_ENV_VAR = "RADAR_HGR_MODEL"


def models_dir() -> Path:
    """Directory holding pre-trained classifier checkpoints."""
    override = os.environ.get(MODELS_ENV_VAR)
    if override:
        return Path(override).expanduser().resolve()
    return PROJECT_DIR / "models"


def radar_profile_dir() -> Path:
    """Directory holding mmWave Studio radar configuration profiles."""
    return PACKAGE_DIR / "radar" / "radar_configure"


#: Checkpoints tried in order when no explicit model path is supplied.
MODEL_CANDIDATES = ("model_2.h5", "model_1.h5")


def resolve_model_path(explicit: Optional[str] = None) -> Optional[Path]:
    """Locate a classifier checkpoint.

    Resolution order: an explicit path, then ``$RADAR_HGR_MODEL``, then the
    known checkpoint names inside :func:`models_dir`.

    Returns
    -------
    Path or None
        The first candidate that exists, or ``None`` when none is available,
        which lets the caller start up without a model rather than crash.
    """
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    env_model = os.environ.get(DEFAULT_MODEL_ENV_VAR)
    if env_model:
        candidates.append(Path(env_model).expanduser())
    candidates.extend(models_dir() / name for name in MODEL_CANDIDATES)

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None
