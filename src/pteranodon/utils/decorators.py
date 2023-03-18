import time
from logging import Logger
from typing import Callable, Optional, Any


def timeit(func: Callable) -> Callable:
    """
    Decorator which can output to stdout or a loggers info stream, the duration it took
    for a function or method to execute.
    Integrated with Classes so if the annotation is used on a method of a class
    it will pull the logger property and use the info stream.
    (Checks first parameter which would be self and pulls .logger and checks type)
    :return Callable: Since it is used as a decorator, returns the function/method wrapped
        with the decorator.
    """

    def inner(func: Callable):
        def time_function(*args, **kwargs) -> None:
            logger: Optional[Logger] = None
            try:  # attempt to pull a logger property off the first arg (would be a self)
                drone: Any = args[0]
                _logger = drone.logger
                logger = _logger if isinstance(_logger, Logger) else None
            except Exception:  # catch basic level exception since multiple could occur
                logger = None
            start_time = time.perf_counter()
            func(*args, **kwargs)
            elapsed_time = time.perf_counter() - start_time
            print_str = f"{func.__name__} took: {round(elapsed_time * 1000, 1)} ms"
            log_func: Callable = logger.info if logger else print  # type: ignore
            log_func(print_str)  # type: ignore

        return time_function

    return inner(func)
