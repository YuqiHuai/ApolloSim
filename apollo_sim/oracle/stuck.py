from shapely.geometry import Point

from apollo_sim.actor import Waypoint
from apollo_sim.sim_env import SimEnv
from apollo_sim.actor import ActorClass
from apollo_sim.oracle import Oracle
from apollo_sim.registry import ORACLE_REGISTRY


@ORACLE_REGISTRY.register('oracle.stuck')
class StuckOracle(Oracle):

    oracle_name = 'oracle.stuck'

    def __init__(
            self,
            idx: str,
            actor: ActorClass,
            sim_env: SimEnv,
            destination: Waypoint,
            destination_threshold: float = 3.0,
            speed_threshold = 0.3,
            max_stuck_time = 90,
            terminate_on_failure: bool = True
    ):
        super(StuckOracle, self).__init__(
            idx=idx,
            actor=actor,
            terminate_on_failure=terminate_on_failure
        )

        self._destination_lane_id = destination.lane.id
        self._destination_point = Point(destination.location.x, destination.location.y)
        self._sim_env = sim_env
        self._destination_threshold = destination_threshold
        self._speed_threshold = speed_threshold
        self._max_stuck_time = max_stuck_time

        self._last_game_time = self._sim_env.game_time # seconds
        self._stuck_time = 0
        self._has_reached = False

    def _tick(self, delta_time: float):

        if self._has_reached:
            return

        actor_speed = self._actor.speed
        if actor_speed <= self._speed_threshold:
            self._stuck_time += (self._sim_env.game_time - self._last_game_time)
        else:
            self._stuck_time = 0

        if self._stuck_time > self._max_stuck_time:
            # check destination
            _map = self._sim_env.map
            actor_lane_id = _map.lane.find_lane_id(x=self._actor.location.x, y=self._actor.location.y)
            if actor_lane_id == self._destination_lane_id:
                dist2dest = self._actor.dist2point(self._destination_point)
                if dist2dest < self._destination_threshold and self._actor.speed < 0.5:
                    self._has_reached = True

            if (not self._has_reached) and self._terminate_on_failure:
                self._termination = True

        self._last_game_time = self._sim_env.game_time