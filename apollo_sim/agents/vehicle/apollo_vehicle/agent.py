import os
import shutil
import time

from loguru import logger
from threading import Thread, Lock

from apollo_sim.actor import ActorClass
from apollo_sim.actor.control import VehicleControl
from apollo_sim.agents.vehicle.apollo_vehicle.publisher import ApolloPublisher
from apollo_sim.sim_env import SimEnv

from apollo_sim.tools import get_instance_logger

class ApolloAgent:

    prefix = 'apollo'

    frequency_localization = 50.0
    frequency_perception = 25.0
    frequency_state = 25.0

    container_record_folder = '/apollo/ApolloSim/records'  # fixed path -> can be moved to flexible one

    _update_lock = Lock()

    def __init__(
            self,
            actor: 'ActorClass',
            sim_env: 'SimEnv',
            trigger_time: float,
            route: list,
            # for recording
            output_folder: str,
            scenario_idx: str = "",
            # other configs
            container_name: str = None,
            debug: bool = False
    ):
        """
        Constructor
        """
        self.actor = actor
        self.sim_env = sim_env
        self.trigger_time = trigger_time
        self.route = route
        self.output_folder = output_folder
        self.scenario_idx = scenario_idx
        self.debug = debug
        self.debug_folder = os.path.join(output_folder, f"debug/{self.prefix}")

        # inner parameters
        self.id = self.actor.id

        # create logger if debug
        if self.debug and self.debug_folder is not None:
            if not os.path.exists(self.debug_folder):
                os.makedirs(self.debug_folder)

            log_file = os.path.join(self.debug_folder, f"{self.prefix}_{self.id}.log")
            self.logger = get_instance_logger(f"{self.prefix}_{self.id}", log_file)
            self.logger.info(f"Logger initialized for {self.prefix}_{self.id}")
        else:
            self.logger = None

        self.publisher = ApolloPublisher(
            self.id,
            trigger_time=self.trigger_time,
            route=self.route,
            container_name=container_name
        )

        # running flag
        self.running = False

        self.thread_chassis_localization = None
        self.thread_perception = None
        self.thread_state = None

    def _tick_chassis_localization(self):
        # TODO: change to game time
        # 0. mix route here
        self.publisher.publish_route(self.sim_env.game_time)
        # 1. publish current state to apollo
        self.publisher.publish_chassis(self.actor)
        self.publisher.publish_localization(self.actor)

    def _tick_perception(self):
        # Update Traffic Information to Apollo
        perception_obs = self.sim_env.actors
        traffic_light_config = self.sim_env.traffic_lights
        self.publisher.publish_traffic_light(traffic_light_config)
        self.publisher.publish_obstacles(perception_obs)

    def _tick_state(self):
        # with self._update_lock:
        agent_control = VehicleControl(
            self.publisher.throttle_percentage,
            self.publisher.brake_percentage,
            self.publisher.steering_percentage
        )
        self.actor.apply_control(agent_control)

    def tick(self):
        self._tick_chassis_localization()
        self._tick_perception()
        self._tick_state()

    def _async_run_localization(self):
        # this is used in async mode
        while (not self.sim_env.termination) and self.running:
            self._tick_chassis_localization()
            time.sleep(1 / self.frequency_localization)

    def _async_run_perception(self):
        # this is used in async mode
        while (not self.sim_env.termination) and self.running:
            self._tick_perception()
            time.sleep(1 / self.frequency_perception)

    def _async_run_state(self):
        # this is used in async mode
        while (not self.sim_env.termination) and self.running:
            self._tick_state()
            time.sleep(1 / self.frequency_state)

    def start(self):
        self.running = True
        self.thread_chassis_localization = Thread(target=self._async_run_localization)
        self.thread_perception = Thread(target=self._async_run_perception)
        self.thread_state = Thread(target=self._async_run_state)

        self.thread_chassis_localization.start()
        self.thread_perception.start()
        self.thread_state.start()

        # start recorder operator
        self.recorder_operator('start', self.container_record_folder, f"{self.scenario_idx}_{self.id}")

    def stop(self):
        self.running = False
        if self.thread_chassis_localization is not None:
            self.thread_chassis_localization.join()
            self.thread_chassis_localization = None
        if self.thread_perception is not None:
            self.thread_perception.join()
            self.thread_perception = None
        if self.thread_state is not None:
            self.thread_state.join()
            self.thread_state = None

        # stop & move recording
        self.recorder_operator('stop')
        local_recording_folder = os.path.join(self.output_folder, f'records_apollo/{self.id}')
        if not os.path.exists(local_recording_folder):
            os.makedirs(local_recording_folder)
        self.move_recording(local_recording_folder, f"{self.container_record_folder}", f"{self.scenario_idx}_{self.id}", delete_flag=True)

        self.publisher.stop()

    ######### Other Tools #########
    def recorder_operator(self, operation, record_folder=None, scenario_id=None):
        self.publisher.recorder_operator(
            operation,
            record_folder=record_folder,
            scenario_id=scenario_id
        )

    def move_recording(self, local_apollo_recording_folder: str, apollo_record_folder: str, scenario_id: str, delete_flag: bool = True):
        # May need parser of recording
        if os.path.exists(local_apollo_recording_folder):
            shutil.rmtree(local_apollo_recording_folder)

        self.publisher.move_recording(
            apollo_record_folder,
            scenario_id,
            local_apollo_recording_folder,
            delete_flag
        )
        logger.info(f'Move Apollo Recording for {self.id} to {local_apollo_recording_folder}')