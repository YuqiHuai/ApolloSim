import math

from typing import Dict
from dataclasses import dataclass, asdict

from apollo_sim.actor.basic.location import Location
from apollo_sim.actor.basic.lane import Lane

@dataclass
class Waypoint:

    lane: Lane
    location: Location
    speed: float

    @classmethod
    def from_json(cls, json_node: Dict) -> 'Waypoint':
        json_node['lane'] = Lane.from_json(json_node['lane'])
        json_node['location'] = Location.from_json(json_node['location'])
        return cls(**json_node)

    def json_data(self):
        return asdict(self)

    def distance(self, other: 'Waypoint'):
        return math.sqrt((self.location.x - other.location.x) ** 2 + (self.location.y - other.location.y) ** 2 + (self.location.z - other.location.z) ** 2)

    ####### Key tools ########
    def get_traffic_light(self):
        pass

    def get_reachable_lanes(self):
        pass