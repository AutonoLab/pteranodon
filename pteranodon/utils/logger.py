import logging
import sys


# setup the logger
def setup_logger(log_file_name: str) -> logging.Logger:
    """
    Creates a logger for the specified log file with the standard format and STDOUT and file handlers

    :param log_file_name: The file name of the log file
    :type log_file_name: str
    :return: The created logger
    :rtype: logging.Logger
    """
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(logging.DEBUG)
    stdout_handler.setFormatter(formatter)

    file_handler = logging.FileHandler(log_file_name)
    file_handler.setLevel(logging.DEBUG)
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
