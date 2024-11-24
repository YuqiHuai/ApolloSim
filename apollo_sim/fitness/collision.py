import numpy as np

from apollo_sim.actor.base import ActorClass
from apollo_sim.fitness import Fitness, register_fitness
from apollo_sim.sim_env import SimEnv

@register_fitness('fitness.collision')
class CollisionFitness(Fitness):
    # min -> collision
    fitness_name = 'fitness.collision'

    def __init__(
            self,
            idx: str,
            actor: ActorClass,
            sim_env: SimEnv
    ):
        super(CollisionFitness, self).__init__(
            idx=idx,
            actor=actor
        )
        self._sim_env = sim_env
        self._min_distance = np.inf

    def _tick(self):

        actors = self._sim_env.actors

        for actor_id, actor in actors.items():
            if actor_id == self._actor.id:
                continue

            dist2agent = self._actor.dist2actor(actor)
            if dist2agent < self._min_distance:
                self._min_distance = dist2agent

        self._result = {
            'min_distance': self._min_distance
        }