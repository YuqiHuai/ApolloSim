import time
from loguru import logger
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
            speed_threshold = 0.3,
            max_stuck_time = 90,
            terminate_on_failure: bool = True
    ):
        super(StuckOracle, self).__init__(
            idx=idx,
            actor=actor,
            terminate_on_failure=terminate_on_failure
        )

        self._sim_env = sim_env
        self._speed_threshold = speed_threshold
        self._max_stuck_time = max_stuck_time

        self._last_game_time = self._sim_env.game_time # seconds
        self._stuck_time = 0

    def tick(self, delta_time: float):
        actor_speed = self._actor.speed
        if actor_speed <= self._speed_threshold:
            self._stuck_time += (self._sim_env.game_time - self._last_game_time)
        else:
            self._stuck_time = 0
        if self._stuck_time > self._max_stuck_time:
            self._termination = True

        self._last_game_time = self._sim_env.game_time