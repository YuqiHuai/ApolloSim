from typing import Dict
from dataclasses import dataclass, asdict

@dataclass
class BoundingBox:

    length: float
    width: float
    height: float

    def json_data(self):
        return asdict(self)

    @classmethod
    def from_json(cls, json_node: Dict) -> 'BoundingBox':
        return cls(**json_node)