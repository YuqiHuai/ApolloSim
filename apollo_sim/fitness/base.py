import copy
from typing import TypeVar, Any
from threading import Lock

from apollo_sim.actor.base import ActorClass

class Fitness(object):

    oracle_name = 'undefined'
    _thread_lock: Lock = Lock()

    def __init__(
            self,
            idx: int or str,
            actor: ActorClass,
            **kwargs
    ):
        self._idx = idx
        self._actor = actor
        self._result: Any = None

    @property
    def id(self):
        return self._idx

    @property
    def result(self):
        return copy.deepcopy(self._result)

    def _tick(self):
        raise NotImplementedError("You should implement this method in subclass.")

    def tick(self):
        with self._thread_lock:
            self._tick()

FitnessClass = TypeVar("FitnessClass", bound=Fitness)