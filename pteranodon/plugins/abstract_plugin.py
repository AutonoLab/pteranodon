from abc import ABC
import asyncio
from asyncio import AbstractEventLoop
from concurrent.futures import Future
from logging import Logger
from collections import deque
from functools import partial
from typing import Tuple, Any, Callable, Optional, Deque, Coroutine
from threading import Condition

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

        self._future_cache: Deque[Future] = deque()
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

    def _future_callback(self, coroutine_name: str, future: Future) -> None:
        """
        Callback associated with each Future scheduled by method _submit_coroutine.
        This will send output to the logger, retrieve results and exceptions, and clear Futures from the cache.
        :param future: A concurrent.future.Future, which has been scheduled with asyncio.run_coroutine_threadsafe
        :return: None
        """
        try:
            self._logger.info(f"Task completed: {coroutine_name} ")
            self._future_cache.remove(future)
            try:
                self._result_cache.append((coroutine_name, future.result()))
            except Exception as e:
                self._logger.error(f"{coroutine_name} -> {e}")
        except Exception as e:
            self._logger.error(f"Callback failed: {future} -> {e}")

    def _submit_coroutine(
        self, coro: Coroutine, callback: Optional[Callable] = None
    ) -> Future:
        """
        Puts a task returned by asyncio.run_coroutine_threadsafe to the future_cache to prevent garbage collection and allow return
        value analysis
        :param coro: A concurrent.future.Future
        :param callback: A Callable, which will be added with add_done_callback to the given coroutine
        :return: The submitted task, if plugin specific callbacks are added
        """
        new_future: Future = asyncio.run_coroutine_threadsafe(coro, loop=self._loop)
        new_future.add_done_callback(partial(self._future_callback, coro.__qualname__))
        if callback is not None:
            new_future.add_done_callback(callback)
        self._future_cache.append(new_future)
        return new_future

    def _submit_generator(self, generator: Callable, retry_time: float = 0.5) -> None:
        """
        Wrapper for the body expressions of the async generators used to read MAVSDK data.
        :param gen_body: A Callable, body of the async generator. Must be callable without arguments (use functools.partial)
        :param retry_time: Attempts to stalls retry of body for this amount of time (ms)
        :return: None, the Futures created by submit_generator are not stable
        """

        async def wrap_generator(generator: Callable, retry_time: float):
            while True:
                try:
                    await generator()
                except grpc.RpcError as rpc_error:
                    if isinstance(
                        rpc_error, grpc.Call
                    ):  # Only "Call" classes (which include _MultiThreadedRendezvous) have code()
                        if rpc_error.code() == grpc.StatusCode.UNAVAILABLE:
                            pass
                    else:
                        raise rpc_error
                finally:
                    await asyncio.sleep(retry_time)

        self._submit_coroutine(wrap_generator(generator, retry_time))

    def _schedule(self, *args: Coroutine) -> None:
        """
        Takes any ammount of coroutins as input and uses add_done_callback to chain all coroutines together
        :param *args: Any amount of coroutines
        :return: The scheduled Future with coroutines chained to it
        """
        coros = [*args]
        # remove the Future if this was called with a callback
        if isinstance(coros[-1], Future):
            _ = coros.pop()
        if len(coros) > 0:
            first = coros[0]
            coros.remove(first)
            self._submit_coroutine(first, partial(self._schedule, *coros))

    def _submit_blocking_coroutine(
        self, coro: Callable, callback: Optional[Callable] = None, timeout: float = 1.0
    ) -> Any:
        """
        Blocks until the given coroutine has completed, returning its result (or None if a timeout occurs)
        :param coro: The Callable Coroutine to run
        :param callback: An optional callback function to call with the coroutine, this is not-blocking
        :param timeout: The maximum number of seconds to block
        :return: The result of the coroutine if it succeeds, otherwise None
        """
        done_condition = Condition()
        blocking_future = self._submit_coroutine(coro(), callback)
        blocking_future.add_done_callback(lambda _: done_condition.notify())

        done_condition.wait(timeout)

        try:
            return blocking_future.result()
        except asyncio.InvalidStateError:
            blocking_future.cancel()
            return None

    def cancel_futures(self) -> None:
        """
        Cancels all currently running (or yet to run) Futures in the queue/cache
        """
        for future in list(self._future_cache):
            future.cancel()
