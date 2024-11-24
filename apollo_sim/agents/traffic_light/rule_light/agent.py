import os
import random
import time
from typing import List
from threading import Thread
from apollo_sim.actor.signal import TrafficLightState
from apollo_sim.agents.traffic_light.rule_light.config import RuleLightConfig
from apollo_sim.sim_env import SimEnv
from apollo_sim.tools import get_instance_logger

class RuleLightAgent:

    prefix = 'rule_light'

    def __init__(
            self,
            config: RuleLightConfig,
            sim_env: SimEnv,
            output_folder: str,
            scenario_idx: str,
            debug: bool = False,
    ):
        self.sim_env = sim_env
        self.config = config
        self.output_folder = output_folder
        self.scenario_idx = scenario_idx
        self.debug = debug
        self.debug_folder = os.path.join(self.output_folder, f"debug/{self.prefix}")

        # Traffic light timing setup from the configuration
        self.green_time = config.green_time
        self.yellow_time = config.yellow_time
        self.red_time = config.red_time
        self.local_random = random.Random(self.config.initial_seed)
        self.traffic_light_actors = self.sim_env.traffic_lights  # dict
        self.traffic_light_keys = list(self.traffic_light_actors.keys())
        self.local_random.shuffle(self.traffic_light_keys)
        self.force_green = config.force_green

        if self.debug:
            if not os.path.exists(self.debug_folder):
                os.makedirs(self.debug_folder)

            log_file = os.path.join(self.debug_folder, f"{self.prefix}.log")
            if os.path.exists(log_file):
                os.remove(log_file)
            self.logger = get_instance_logger(f"{self.prefix}", log_file)
            self.logger.info(f"Logger initialized for {self.prefix}")

        self.running = False
        self.thread_run = None

        # other flags
        self.scenario_idx = None
        self.initialize()

    def initialize(self):
        if self.force_green:
            for traffic_light_id in self.traffic_light_keys:
                traffic_light_actor = self.traffic_light_actors[traffic_light_id]
                traffic_light_actor.update_state(TrafficLightState.green)
                if self.debug:
                    self.logger.info(f"Set traffic light {traffic_light_id} to GREEN (forced)")
        else:
            complete_lights = []
            for traffic_light_id in self.traffic_light_keys:
                if traffic_light_id in complete_lights:
                    continue
                traffic_light_actor = self.traffic_light_actors[traffic_light_id]
                traffic_light_actor.update_state(TrafficLightState.green)
                complete_lights.append(traffic_light_id)
                if self.debug:
                    self.logger.info(f"Set traffic light {traffic_light_id} to GREEN")

                # Set conflicting lights to red
                for conflict_id in traffic_light_actor.conflicts:
                    if conflict_id in complete_lights:
                        continue
                    conflict_light = self.traffic_light_actors[conflict_id]
                    conflict_light.update_state(TrafficLightState.red)
                    complete_lights.append(conflict_id)
                    if self.debug:
                        self.logger.info(f"Set traffic light {conflict_id} (conflict with {traffic_light_id}) to RED")

    ######## tick ########
    def tick(self) -> List[dict]:
        if self.force_green:
            return []

        complete_lights = []
        current_game_time = self.sim_env.game_time  # Use the global game time directly

        for traffic_light_id in self.traffic_light_keys:
            if traffic_light_id in complete_lights:
                continue

            traffic_light_actor = self.traffic_light_actors[traffic_light_id]
            current_state = traffic_light_actor.get_state()
            last_state_time = traffic_light_actor.get_last_state_time()  # When the state was last updated
            elapsed_time = current_game_time - last_state_time

            if self.debug:
                self.logger.info(
                    f"Traffic light {traffic_light_id} state: {current_state}, "
                    f"elapsed_time: {elapsed_time:.2f}, game_time: {current_game_time:.2f}"
                )

            # State transitions based on the global game time
            if current_state == TrafficLightState.green and elapsed_time >= self.green_time:
                traffic_light_actor.update_state(TrafficLightState.yellow)
                traffic_light_actor.set_last_state_time(current_game_time)
                if self.debug:
                    self.logger.info(f"Traffic light {traffic_light_id} transitioned from GREEN to YELLOW")

            elif current_state == TrafficLightState.yellow and elapsed_time >= self.yellow_time:
                traffic_light_actor.update_state(TrafficLightState.red)
                traffic_light_actor.set_last_state_time(current_game_time)
                if self.debug:
                    self.logger.info(f"Traffic light {traffic_light_id} transitioned from YELLOW to RED")

            elif current_state == TrafficLightState.red and elapsed_time >= self.red_time:
                if not any(self.traffic_light_actors[conflict_id].get_state() == TrafficLightState.green
                           for conflict_id in traffic_light_actor.conflicts):
                    traffic_light_actor.update_state(TrafficLightState.green)
                    traffic_light_actor.set_last_state_time(current_game_time)
                    if self.debug:
                        self.logger.info(f"Traffic light {traffic_light_id} transitioned from RED to GREEN")
                else:
                    if self.debug:
                        self.logger.info(f"Traffic light {traffic_light_id} remains RED due to conflicts")

            complete_lights.append(traffic_light_id)


    ######## async run ########
    def _async_run(self):
        while (not self.sim_env.termination) and self.running:
            self.tick()
            time.sleep(0.01)

    def start(self):
        self.running = True
        self.thread_run = Thread(target=self._async_run)
        self.thread_run.start()

    def stop(self):
        self.running = False
        if self.thread_run is not None:
            self.thread_run.join()
            self.thread_run = None