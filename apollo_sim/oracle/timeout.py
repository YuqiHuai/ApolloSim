import time

from apollo_sim.oracle import Oracle, register_oracle
from apollo_sim.sim_env import SimEnv


@register_oracle('oracle.timeout')
class TimeoutOracle(Oracle):

    oracle_name = 'oracle.timeout'

    def __init__(
            self,
            idx: str,
            sim_env: SimEnv,
            time_limit = 300,
            terminate_on_failure: bool = True
    ):
        super(TimeoutOracle, self).__init__(
            idx=idx,
            actor=None,
            terminate_on_failure=terminate_on_failure
        )
        self._sim_env = sim_env
        self._time_limit = time_limit
        self._start_game_time = None

    def tick(self):
        if self._start_game_time is None:
            self._start_game_time = self._sim_env.game_time

        spend_time = self._sim_env.game_time - self._start_game_time

        if spend_time > self._time_limit:
            self._termination = True