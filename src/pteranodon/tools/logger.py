import logging
import sys
from typing import Optional


# setup the logger
def setup_logger(
    log_level: int = logging.INFO,
    log_stdout_level: int = logging.DEBUG,
    log_file_level: int = logging.DEBUG,
    log_file_name: Optional[str] = None,
) -> logging.Logger:
    """
    Creates a logger for the specified log file with the standard format and STDOUT and file handlers

    :param log_file_name: The file name of the log file
    :type log_file_name: str
    :return: The created logger
    :rtype: logging.Logger
    """
    logger = logging.getLogger()
    logger.setLevel(log_level)
    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(log_stdout_level)
    stdout_handler.setFormatter(formatter)

    if log_file_name is not None:
        file_handler = logging.FileHandler(log_file_name)
        file_handler.setLevel(log_file_level)
        file_handler.setFormatter(formatter)

        logger.addHandler(file_handler)
        logger.addHandler(stdout_handler)

    return logger


def close_logger(logger: logging.Logger) -> None:
    """
    Close the handlers added to the given logger

    :param logger: The logger to remove handlers for
    :type logger: logging.Logger
    """
    handlers = logger.handlers[:]
    for handler in handlers:
        logger.removeHandler(handler)
        handler.close()
