"""Application-wide logging setup.

The library modules only ever call ``logging.getLogger(__name__)``; the handler
and level are chosen once, here, by whichever front end is running.
"""

import logging

LOG_FORMAT = "%(asctime)s %(levelname)-8s %(name)s: %(message)s"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def configure_logging(verbose: bool = False) -> None:
    """Install a console handler on the root logger.

    Parameters
    ----------
    verbose:
        Emit ``DEBUG`` records, which include the radar protocol exchange,
        instead of ``INFO`` and above.
    """
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format=LOG_FORMAT,
        datefmt=DATE_FORMAT,
    )
