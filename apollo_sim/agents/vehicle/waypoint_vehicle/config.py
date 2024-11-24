from dataclasses import dataclass, asdict
from typing import List, Dict

from apollo_sim.actor import Waypoint, Location

def get_basic_config():
    return {
        "max_speed": 25.0,
        "max_speed_junction": 10.0,
        "max_acceleration": 6.0,
        "max_deceleration": -6.0,
        "max_steering": 0.8,
        "collision_threshold": 5.0,
        "ignore_vehicle": False,
        "ignore_walker": False,
        "ignore_static_obstacle": False,
        "ignore_traffic_light": False,
        "min_distance": 2.0, # to filter next waypoint
        "finish_buffer": 20.0,
        "collision_distance_threshold": 5.0,
        "pid_lateral_cfg": {
            'K_P': 1.05,
            'K_D': 0.01,
            'K_I': 0.01,
        },
        "pid_longitudinal_cfg": {
            'K_P': 1.0,
            'K_D': 0,
            'K_I': 0.05,
        },
        "remove_after_finish" : False
    }

@dataclass
class WaypointVehicleConfig(object):

    idx: int
    category: str
    behavior: List[Waypoint]
    trigger_time: float = 0.0
    role: str = 'vehicle'

    def __init__(
            self,
            idx: int,
            category: str,
            behavior: List[Waypoint], # start location point
            trigger_time: float = 0.0,
            role: str = 'vehicle'
    ):
        self.idx = idx
        self.category = category
        self.behavior = behavior
        self.trigger_time = trigger_time
        self.role = role

    @property
    def location(self) -> Location:
        return self.behavior[0].location

    @classmethod
    def from_json(cls, json_node: Dict) -> 'WaypointVehicleConfig':
        behavior = list()
        for r_i, r_js in enumerate(json_node['behavior']):
            behavior.append(Waypoint.from_json(r_js))
        json_node['behavior'] = behavior
        return cls(**json_node)

    def json_data(self) -> Dict:
        return asdict(self)