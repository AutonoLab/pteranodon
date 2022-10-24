import functools
from abc import ABC, abstractmethod
import asyncio
from asyncio import AbstractEventLoop
from concurrent import futures
from logging import Logger
from collections import deque, defaultdict
from functools import partial, cached_property
import time
from inspect import signature
from typing import (
    Tuple,
    Any,
    Callable,
    Optional,
    Deque,
    Coroutine,
    Dict,
    List,
)

import grpc
from mavsdk import System


class AbstractPlugin(ABC):
    """
    Base plugin functionality, no methods required to overwrite
    """

    def __init__(
        self, name: str, system: System, loop: AbstractEventLoop, logger: Logger
    ) -> None:
        self._name: str = name
        self._system: System = system
        self._loop: AbstractEventLoop = loop
        self._logger: Logger = logger

        self._ready: bool = False
        self._num_generators: int = 0

        self._future_cache: Deque[futures.Future] = deque()
        self._result_cache: Deque[Tuple[str, Any]] = deque(maxlen=10)

        self._async_gen_data: Dict[Callable, Optional[Any]] = defaultdict(lambda: None)
        self._async_handlers: Dict[Callable, List[Callable]] = defaultdict(list)
        self._async_rate_data: Dict[Callable, float] = defaultdict(lambda: 1.0)
        self._rate_last_times: Dict[Callable, float] = {}

        self._stopped = False

    #     while True:
    #         try:
    #             self._init()
    #             self._end_init()
    #         except Exception:
    #             self._logger.error(f"Failed to start class: {self._name}, retrying...")
    #             self.cancel_futures()
    #             self._result_cache.clear()
    #             self._future_cache.clear()

    # @abstractmethod
    # def _init(self):
    #     pass

    def _register_handler(self, generator: Callable):
        """
        Annotation to register a handler for an async generator that is submitted via submit_simple_generator

        :param generator: The generator to trigger the handler for
        :type generator: Callable
        """

        def inner(func):
            self._async_handlers[generator].append(func)
            return func

        return inner

    @property
    def name(self) -> str:
        """
        :return: str ; returns the name of the plugin as a string
        """
        return self._name

    @cached_property
    def num_generators(self) -> int:
        """
        :return: int ; returns the number of generators this plugin starts
        """
        # get the number of generators on the plugin
        self._num_generators = len([
            func
            for func in dir(self)
            if not func.startswith("_") and "register" in func and "handler" in func
        ])
        return self._num_generators

    @property
    def ready(self) -> bool:
        """
        :return: bool ; returns the number of generators
        """
        return self._ready

    def _end_init(self) -> None:
        """
        Method which should be called at the end of the __init__ method for each class which inherits this
        """
        # call to wait on sleep so that way generators get created
        self._loop.run_until_complete(asyncio.sleep(0.05))
        self._ready = True

    def _future_callback(
        self, coroutine_name: str, is_gen: bool, future: futures.Future
    ) -> None:
        """
        Callback associated with each Future scheduled by method _submit_coroutine.
        This will send output to the logger, retrieve results and exceptions, and clear Futures from the cache.

        :param coroutine_name: The name of the coroutine that finished
        :type coroutine_name: str
        :param is_gen: Whether the coroutine is a generator or not (used for logging)
        :type is_gen: bool
        :param future: A concurrent.future.Future, which has been scheduled with asyncio.run_coroutine_threadsafe
        :type future: concurrent.future.Future
        :return: None
        """

        try:
            self._future_cache.remove(future)

            if is_gen:
                return

            self._logger.info(f"Task completed: {coroutine_name} ")

            try:
                self._result_cache.append((coroutine_name, future.result()))
            except Exception as e:
                if len(str(e)) > 0:
                    self._logger.error(f"{coroutine_name} -> {e}")
        except Exception as e:
            self._logger.error(f"Callback failed: {coroutine_name} -> {e}")

    def _submit_coroutine(
        self,
        coro: Coroutine,
        callback: Optional[Callable] = None,
        is_generator: bool = False,
    ) -> futures.Future:
        """
        Puts a future returned by asyncio.run_coroutine_threadsafe to the future_cache to prevent garbage collection and allow return
        value analysis
        :param coro: The coroutine to run
        :type coro: asyncio.Coroutine
        :param callback: A function to call once the coroutine finishes
        :type callback: Optional[Callable]
        :param is_generator: Whether the passed coroutine is a generator or not
         (DO NOT pass generators, call submit_generator to submit a generator)
        :type is_generator: bool
        :return: The Future object generated run to the coroutine
        :rtype: concurrent.futures.Future
        """

        new_future: futures.Future = asyncio.run_coroutine_threadsafe(
            coro, loop=self._loop
        )
        new_future.add_done_callback(
            partial(self._future_callback, coro.__qualname__, is_generator)
        )
        if callback is not None:
            new_future.add_done_callback(callback)
        self._future_cache.append(new_future)
        return new_future

    def _submit_simple_generator(
        self,
        generator: Callable,
        should_compute_rate: bool = False,
        retry_time: float = 0.5,
    ) -> futures.Future:
        """
        Wrapper for the body expressions of the async generators used to read MAVSDK data.
         This function will automatically handle the saving of the generated data, any added handlers, and rate calculations.
         _submit_generator does not add this functionality.
        :param generator: The Callable to submit and handle
        :type generator: Callable
        :param should_compute_rate: Whether the rate of updates should be computed for this property (in Hz)
        :type should_compute_rate: bool
        :param retry_time: Attempts to stall retry of body for this amount of time (ms)
        :type retry_time: float
        :return: The future created from the submit_coroutine call of wrap_generator
        """

        async def async_gen_wrapper(gen: Callable, comp_rate: bool = False) -> None:
            async for data in gen():
                self._async_gen_data[gen] = data

                if comp_rate:
                    current_time = time.perf_counter()
                    try:

                        prev_time = self._rate_last_times[gen]

                        delta_secs = current_time - prev_time
                        # Average the current value and the last value if they are close enough to account for minor variations
                        new_hz = 1 / delta_secs
                        current_hz = self._async_rate_data[gen]
                        if abs(new_hz - current_hz) <= 0.5:
                            new_hz = (new_hz + current_hz) / 2
                        self._async_rate_data[gen] = new_hz

                    except KeyError:
                        # Need one cycle to calculate
                        self._rate_last_times[gen] = current_time
                        pass

                func_handlers = self._async_handlers[gen]
                if len(func_handlers) > 0:
                    for handler in func_handlers:
                        sig = signature(handler)
                        params = list(sig.parameters.keys())

                        if len(params) <= 0:
                            handler()
                            continue

                        # Some handlers may use "self", don't forget to include it.
                        args_list = [data]
                        if params[0] == "self":
                            args_list.insert(0, self)

                        if len(sig.parameters.keys()) in [1, 2]:
                            handler(*args_list)
                            continue

                        self._logger.error(
                            f"Could not run handler ({handler.__qualname__}) for async generator ({gen.__qualname__})! Too many arguments!"
                        )

        return self._submit_generator(
            functools.partial(async_gen_wrapper, generator, should_compute_rate),
            retry_time=retry_time,
        )

    def _submit_generator(
        self, generator: Callable, retry_time: float = 0.5
    ) -> futures.Future:

        """
        Wrapper for the body expressions of the async generators used to read MAVSDK data.

        :param generator: Body of the async generator. Must be callable without arguments (use functools.partial otherwise)
        :type generator: Callable
        :param retry_time: Attempts to stall retry of body for this amount of time (ms)
        :type retry_time: float
        :return: The future created from the submit_coroutine call of wrap_generator
        """

        async def wrap_generator(gen: Callable, ret_time: float):
            while True:
                try:
                    await gen()
                except grpc.RpcError as rpc_error:
                    if isinstance(
                        rpc_error, grpc.Call
                    ):  # Only "Call" classes (which include _MultiThreadedRendezvous) have code()
                        if rpc_error.code() == grpc.StatusCode.UNAVAILABLE:
                            pass
                    else:
                        raise rpc_error
                finally:
                    await asyncio.sleep(ret_time)

        return self._submit_coroutine(
            wrap_generator(generator, retry_time), is_generator=True
        )

    def _schedule(self, *args: Coroutine):
        """
        Takes any amount of coroutines as input and runs them sequentially using add_done_callback

        :param args: Any amount of coroutines
        :type args: *Coroutine
        """

        coros = [*args]
        # remove the Future if this was called with a callback
        if isinstance(coros[-1], futures.Future):
            _ = coros.pop()
        if len(coros) > 0:
            first = coros[0]
            coros.remove(first)
            self._submit_coroutine(first, partial(self._schedule, *coros))

    def _submit_blocking_coroutine(
        self, coro: Coroutine, callback: Optional[Callable] = None, timeout: float = 1.0
    ) -> Optional[Any]:
        """
        Blocks until the given coroutine has completed, returning its result (or None if a timeout occurs)

        :param coro: The coroutine to run
        :type coro: Coroutine
        :param callback: An optional callback function to call with the coroutine, this is non-blocking
        :type callback: Optional[Callable]
        :param timeout: The maximum number of seconds to block
        :type timeout: float
        :return: The result of the coroutine if it succeeds, otherwise None
        :rtype: Optional[Any]
        """
        blocking_future = self._submit_coroutine(coro, callback)

        try:
            return blocking_future.result(timeout=timeout)
        except futures.TimeoutError:
            return None

    def cancel_futures(self) -> None:
        """
        Cancels all currently running (or yet to run) Futures in the queue/cache
        """
        for future in list(self._future_cache):
            future.cancel()

    def get_results(self) -> List[Any]:
        """
        Gets the current items in the results cache as a list
        """
        return list(self._result_cache)

    def clear_results(self) -> None:
        """
        Clears the results cache
        """
        self._result_cache.clear()

    def get_newest_result(self) -> Any:
        """
        Gets the last result added to the result cache
        """
        return list(self._result_cache)[-1] if len(self._result_cache) > 0 else None

    def get_oldest_result(self) -> Any:
        """
        Gets the oldest result added to the result cache
        """
        return list(self._result_cache)[0] if len(self._result_cache) > 0 else None
