import copy

from threading import Lock
from typing import TypeVar

class Actor(object):

    id: int or str = 0 # only support int id
    category: str = 'unknown'
    role: str = 'unknown'

    _thread_lock: Lock = Lock()

    def __init__(
            self,
            id: int or str,
            role: str,
            **kwargs
    ):
        if role not in ['ads', 'traffic_light', 'walker', 'vehicle', 'static']:
            raise ValueError(f"Invalid role {role}, please set in the range of ['ads', 'traffic_light', 'walker', 'vehicle', 'static']")

        self.id = id
        self.role = role

    def _json_data(self):
        return {
            "id": self.id,
            "category": self.category,
            "role": self.role
        }

    def json_data(self):
        with self._thread_lock:
            return copy.deepcopy(self._json_data())

    def _tick(self, delta_time: float):
        pass

    def tick(self, delta_time: float):
        with self._thread_lock:
            self._tick(delta_time)

ActorClass = TypeVar("ActorClass", bound=Actor)