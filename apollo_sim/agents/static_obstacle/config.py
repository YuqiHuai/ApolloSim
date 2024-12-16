from __future__ import annotations

from typing import List, Dict
from dataclasses import dataclass, asdict

from apollo_sim.actor import Location, Waypoint


@dataclass
class StaticObstacleConfig(object):

    idx: int
    category: str
    waypoint: Waypoint
    role: str = 'static'

    def __init__(
            self,
            idx: int,
            category: str,
            waypoint: Waypoint,
            role: str = 'static',
    ):
        self.idx = idx
        self.category = category
        self.waypoint = waypoint
        self.role = role

    @property
    def location(self) -> Location:
        return self.waypoint.location

    @classmethod
    def from_json(cls, json_node: Dict) -> 'StaticObstacleConfig':
        json_node['waypoint'] = Waypoint.from_json(json_node['waypoint'])
        return cls(**json_node)

    def json_data(self) -> Dict:
        return asdict(self)