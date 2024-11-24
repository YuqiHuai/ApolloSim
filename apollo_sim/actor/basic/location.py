from dataclasses import dataclass, asdict
from typing import Dict

@dataclass
class Location:

    x: float
    y: float
    z: float
    pitch: float
    yaw: float # heading
    roll: float

    @property
    def heading(self):
        return self.yaw

    def json_data(self):
        return asdict(self)

    @classmethod
    def from_json(cls, json_node: Dict) -> 'Location':
        return cls(**json_node)

    def distance(self, other: 'Location') -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5