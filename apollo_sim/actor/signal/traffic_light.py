from typing import List
from enum import Enum

from apollo_sim.registry import ACTOR_REGISTRY
from apollo_sim.actor.base import Actor

class TrafficLightState(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
    """
    green = 0
    yellow = 1
    red = 2
    unknown = 3

@ACTOR_REGISTRY.register("signal.traffic_light")
class TrafficLight(Actor):

    id: int or str = 0
    # basic information - fixed
    category: str = 'signal.traffic_light'

    def __init__(
            self,
            id: int or str,
            role: str,
            stop_line: List[List[float]],
            state: TrafficLightState = TrafficLightState.green,
            conflicts: List[str] = None,
            equals: List[str] = None
    ):
        # Note that the idx should align with the map id
        super(TrafficLight, self).__init__(id, role)
        self.state = state
        self.stop_line = stop_line
        self.conflicts = conflicts
        self.light_count = 0
        self.equals = equals
        self.state = state
        self.timer = 0.0

    def update_state(self, state: str):
        with self._thread_lock:
            self.state = state

    def get_state(self):
        with self._thread_lock:
            return self.state

    def get_last_state_time(self):
        with self._thread_lock:
            return self.timer

    def set_last_state_time(self, update_time: float):
        with self._thread_lock:
            self.timer = update_time

    def reset_timer(self):
        with self._thread_lock:
            self.timer = 0.0

    def _json_data(self):
        return {
            'id': self.id,
            'category': self.category,
            'state': self.state.name,
            'role': self.role,
            'stop_line': self.stop_line,
            'conflicts': self.conflicts,
            'equals': self.equals
        }

