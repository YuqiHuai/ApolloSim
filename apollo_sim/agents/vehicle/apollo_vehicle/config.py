from dataclasses import dataclass, asdict
from typing import Any, List, Dict

from apollo_sim.actor import Waypoint, Location

@dataclass
class ApolloConfig(object):

    idx: Any
    category: str
    route: List[Waypoint]
    trigger_time: float
    role: str = 'ads'

    def __init__(
            self,
            idx: Any,
            category: str,
            route: List[Waypoint],
            trigger_time: float,
            role: str = 'ads'
    ):
        self.idx = idx
        self.category = category
        self.route = route
        self.trigger_time = trigger_time
        self.role = role

    @property
    def location(self) -> Location:
        return self.route[0].location

    @classmethod
    def from_json(cls, json_node: Any):
        json_route = json_node["route"]
        route = list()
        for item in json_route:
            route.append(Waypoint.from_json(item))
        json_node["route"] = route
        return cls(**json_node)

    def json_data(self) -> Dict:
        return asdict(self)