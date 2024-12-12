import numpy as np

from apollo_sim.sim_env import SimEnv
from apollo_sim.actor import ActorClass
from apollo_sim.fitness import Fitness
from apollo_sim.registry import FITNESS_REGISTRY

@FITNESS_REGISTRY.register('fitness.destination')
class DestinationFitness(Fitness):
    """
    Two parts:
    1. stuck time -> max is better
    2. travel distance -> min is better
    """
    oracle_name = 'fitness.destination'

    def __init__(
            self,
            idx: str,
            actor: ActorClass,
            sim_env: SimEnv,
            threshold_speed: float = 0.5,
    ):
        super(DestinationFitness, self).__init__(
            idx=idx,
            actor=actor
        )

        self._sim_env = sim_env
        self._speed_threshold = threshold_speed

        self._last_game_time = self._sim_env.game_time # seconds
        self._stuck_time = 0
        self._travel_distance = 0
        self._last_position = self._actor.location

        self._max_stuck_time = -np.inf

    def tick(self):

        # travel distance
        self._travel_distance += self._last_position.distance(self._actor.location)
        self._last_position = self._actor.location

        # stuck time
        actor_speed = self._actor.speed
        if actor_speed <= self._speed_threshold:
            self._stuck_time += (self._sim_env.game_time - self._last_game_time)
        else:
            self._stuck_time = 0
        self._last_game_time = self._sim_env.game_time

        if self._stuck_time > self._max_stuck_time:
            self._max_stuck_time = self._stuck_time

        self._result = {
            'stuck_time': self._stuck_time,
            'travel_distance': self._travel_distance,
        }