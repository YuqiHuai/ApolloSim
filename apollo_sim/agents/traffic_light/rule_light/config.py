from dataclasses import dataclass, asdict
from typing import Dict

@dataclass
class RuleLightConfig:

    green_time: float
    yellow_time: float
    red_time: float
    initial_seed: int
    force_green: bool

    def __init__(
            self,
            green_time: float = 10.0,
            yellow_time: float = 3.0,
            red_time: float = 10.0,
            initial_seed: int = 0,
            force_green: bool = False
    ):
        self.green_time = green_time
        self.yellow_time = yellow_time
        self.red_time = red_time
        self.initial_seed = initial_seed
        self.force_green = force_green

    def json_data(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_json(cls, json_node: Dict) -> 'RuleLightConfig':
        return cls(**json_node)