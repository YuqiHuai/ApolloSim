import os
import time

from threading import Thread

from apollo_sim.actor import ActorClass
from apollo_sim.agents.static_obstacle.config import StaticObstacleConfig
from apollo_sim.sim_env import SimEnv
from apollo_sim.tools import get_instance_logger

class StaticObstacleAgent(object):

    frequency = 25.0

    prefix = 'static_obstacle'

    def __init__(
            self,
            actor: ActorClass,
            config: StaticObstacleConfig,
            sim_env: SimEnv,
            output_folder: str,
            scenario_idx: str = "",
            debug: bool = False,
    ):
        self.actor = actor
        self.config = config
        self.sim_env = sim_env
        self.output_folder = output_folder
        self.scenario_idx = scenario_idx
        self.debug = debug
        self.debug_folder = os.path.join(output_folder, f"debug/{self.prefix}")

        # create logger if debug
        if self.debug and self.debug_folder is not None:
            if not os.path.exists(self.debug_folder):
                os.makedirs(self.debug_folder)

            log_file = os.path.join(self.debug_folder, f"{self.prefix}_{self.actor.id}.log")
            self.logger = get_instance_logger(f"{self.prefix}_{self.config.idx}", log_file)
            self.logger.info(f"Logger initialized for {self.prefix}_{self.config.idx}")
        else:
            self.logger = None

        self.running = False
        self.thread_run = None

        # other flags
        self.scenario_idx = None

    # tick is update location
    def tick(self):
        # Each agent should have its own time interval, which is independent of the simulation time interval.
        self.actor.update_location(self.config.waypoint.location)

    def _async_run(self):
        while (not self.sim_env.termination) and self.running:
            self.tick()
            time.sleep(1 / self.frequency)

    def start(self):
        self.running = True
        self.thread_run = Thread(target=self._async_run)
        self.thread_run.setDaemon(True)
        self.thread_run.start()

    def stop(self):
        self.running = False
        if self.thread_run is not None:
            self.thread_run.join()
            self.thread_run = None