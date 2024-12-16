import numpy as np

from apollo_sim.actor.base import ActorClass
from apollo_sim.oracle import Oracle
from apollo_sim.sim_env import SimEnv
from apollo_sim.registry import ORACLE_REGISTRY


@ORACLE_REGISTRY.register('oracle.collision')
class CollisionOracle(Oracle):

    oracle_name = 'oracle.collision'

    def __init__(
            self,
            idx: str,
            actor: ActorClass,
            sim_env: SimEnv,
            terminate_on_failure: bool = True,
            threshold: float = 0.001
    ):
        super(CollisionOracle, self).__init__(
            idx=idx,
            actor=actor,
            terminate_on_failure=terminate_on_failure
        )
        self._sim_env = sim_env
        self._threshold = threshold
        self._min_distance = np.inf

    def _tick(self, delta_time: float):

        actors = self._sim_env.actors

        for actor_id, actor in actors.items():
            if actor_id == self._actor.id:
                continue

            dist2agent = self._actor.dist2actor(actor)
            if dist2agent < self._min_distance:
                self._min_distance = dist2agent

            if self._min_distance <= self._threshold:
                if self._terminate_on_failure:
                    self._termination = True
                    break