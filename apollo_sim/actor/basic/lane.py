from dataclasses import dataclass, asdict
from typing import Dict

@dataclass
class Lane:

    id: str
    s: float
    l: float

    def json_data(self):
        return asdict(self)

    @classmethod
    def from_json(cls, json_node: Dict) -> 'Lane':
        return cls(**json_node)