"""
Multiprocessing-safe logging module
Provides a simple logging interface compatible with multiprocessing contexts
"""

import logging
import sys

def basic_config(level=logging.INFO):
    """
    Configure basic logging settings for the entire application

    Args:
        level: Logging level (default: INFO)
    """
    logging.basicConfig(
        level=level,
        format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        stream=sys.stdout
    )

def get_logger(name, level=logging.INFO):
    """
    Get a logger instance configured for multiprocessing

    Args:
        name: Logger name (typically __name__)
        level: Logging level (default: INFO)

    Returns:
        Logger instance
    """
    logger = logging.getLogger(name)

    # Only configure if not already configured
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        formatter = logging.Formatter(
            '[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(level)
        logger.propagate = False  # Prevent duplicate logs

    return logger

# Logging level constants for convenience
DEBUG = logging.DEBUG
INFO = logging.INFO
WARNING = logging.WARNING
ERROR = logging.ERROR
CRITICAL = logging.CRITICAL
