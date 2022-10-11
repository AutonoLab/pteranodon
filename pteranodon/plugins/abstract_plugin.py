from abc import ABC
import asyncio
from asyncio import AbstractEventLoop
from concurrent import futures
from logging import Logger
from collections import deque
from functools import partial
from typing import Tuple, Any, Callable, Optional, Deque, Coroutine

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

        self._future_cache: Deque[futures.Future] = deque()
        self._result_cache: Deque[Tuple[str, Any]] = deque(maxlen=10)

        self._stopped = False

    @property
    def name(self) -> str:
        """
        :return: str ; returns the name of the plugin as a string
        """
        return self._name

    def _end_init(self) -> None:
        """
        Method which should be called at the end of the __init__ method for each class which inherits this
        """
        # call to wait on sleep so that way generators get created
        self._loop.run_until_complete(asyncio.sleep(0.05))

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
