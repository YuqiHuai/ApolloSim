from typing import TypeVar
from threading import Lock

from apollo_sim.actor.base import ActorClass

class Oracle(object):

    oracle_name = 'undefined'
    _thread_lock: Lock = Lock()

    def __init__(
            self,
            idx: int or str,
            actor: ActorClass,
            terminate_on_failure: bool = False,
            **kwargs
    ):
        self._idx = idx
        self._actor = actor
        self._terminate_on_failure = terminate_on_failure
        self._termination = False

    @property
    def id(self):
        return self._idx

    @property
    def termination(self):
        return self._termination

    def _tick(self, delta_time: float):
        raise NotImplementedError("You should implement this method in subclass.")

    def tick(self, delta_time: float):
        with self._thread_lock:
            self._tick(delta_time)

OracleClass = TypeVar("OracleClass", bound=Oracle)