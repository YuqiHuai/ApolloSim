import numpy as np

from shapely.geometry import Point

from apollo_sim.sim_env import SimEnv
from apollo_sim.actor import Waypoint
from apollo_sim.actor.base import ActorClass
from apollo_sim.oracle import Oracle, register_oracle

@register_oracle('oracle.destination')
class DestinationOracle(Oracle):

    oracle_name = 'oracle.destination'

    def __init__(
            self,
            idx: str,
            actor: ActorClass,
            destination: Waypoint,
            sim_env: SimEnv,
            threshold: float = 3.0,
            terminate_on_failure: bool = True # Note: in this situation, it means termination on reaching destination
    ):
        super(DestinationOracle, self).__init__(
            idx=idx,
            actor=actor,
            terminate_on_failure=terminate_on_failure
        )

        self._destination_lane_id = destination.lane.id
        self._destination_point = Point(destination.location.x, destination.location.y)
        self._sim_env = sim_env
        self._threshold = threshold
        self._min_distance = np.inf

    def tick(self):
        _map = self._sim_env.map
        actor_lane_id, _ = _map.lane.get_lane(self._actor.location)
        if actor_lane_id == self._destination_lane_id:
            dist2dest = self._actor.dist2point(self._destination_point)
            if dist2dest < self._threshold and self._actor.speed < 0.5:
                self._termination = True # reach the destination