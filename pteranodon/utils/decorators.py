import time
from logging import Logger
from typing import Callable, Optional


def timeit() -> Callable:
    """
    Decorator which can output to stdout or a loggers info stream, the duration it took 
    for a function or method to execute
    :param logger: An optional Logger instance to use for logging the message. If None uses stdout
    :return Callable: Since it is used as a decorator, returns the function/method wrapped 
        with the decorator.
    """
    def inner(func: Callable):
        def time_function(*args, **kwargs) -> None:
            logger: Optional[Logger] = None
            try:  # attempt to pull a logger property off the first arg (would be a self)
                _logger = args[0].logger
                logger = _logger if isinstance(_logger, Logger) else None
            except Exception:  # catch basic level exception since multiple could occur
                pass
            start_time = time.perf_counter()
            func(*args, **kwargs)
            elapsed_time = time.perf_counter() - start_time
            print_str = f"{func.__name__} took: {round(elapsed_time * 1000, 1)} ms"
            log_func = logger.info if logger else print
            log_func(print_str)
        return time_function
    return inner
