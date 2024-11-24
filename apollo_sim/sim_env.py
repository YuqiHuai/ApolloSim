import json
import os
import time
import multiprocessing

from concurrent.futures import ThreadPoolExecutor
from threading import Thread, Lock
from typing import Dict, Optional
from loguru import logger

from apollo_sim.actor.signal.traffic_light import TrafficLight
from apollo_sim.tools.global_data import GlobalData, Timer, Map
from apollo_sim.tools import Recorder
from apollo_sim.vis.render import SimRender

class SimEnv(object):
    """
    This is used to control all actors in the scenario
    Each use is re-create
    """
    def __init__(
            self,
            map_name: str,
            apollo_root: str,
            map_root: str,
            sim_frequency: float = 25.0,
            save_record: bool = True,
            off_screen: bool = False,
            port: int = 5000,
            debug: bool = False
    ):
        # setup parameters
        GlobalData.map_name = map_name
        GlobalData.apollo_root = apollo_root
        GlobalData.map_root = map_root
        GlobalData.sim_frequency = sim_frequency

        self._use_record = save_record
        self._use_render = not off_screen
        self._port = port
        self._debug = debug

        # Other parameters

        # thread management
        self._thread_run = None
        # lock
        self._lock_actor_manage = Lock()
        self._lock_traffic_light_manage = Lock()
        self._lock_oracle_manage = Lock()
        self._lock_fitness_manage = Lock()

        # flags
        self._termination = False
        self._render_first = False

        # vis & record
        self._recorder = None  # Record simulation data
        self._render = None

        # Render config & pipe
        self._render_parent_pipe = None
        self._render_child_pipe = None

        self._actor_pool = dict()
        self._traffic_light_pool = dict() # model traffic light
        self._oracle_pool = dict() # oracle
        self._fitness_pool = dict() # fitness

        GlobalData.timer = Timer(fps=GlobalData.sim_frequency)

        self._initialize()

    def _initialize(self, reload_render = True):
        GlobalData.map = Map()
        GlobalData.map.load_from_file(os.path.join(GlobalData.map_root, GlobalData.map_name))

        # register traffic light actors - default is green
        traffic_lights = GlobalData.map.traffic_light.get_all() # ids
        for tl_id in traffic_lights:
            stop_line = GlobalData.map.traffic_light.get_stop_line(tl_id)
            conflicted_lights, equal_lights = GlobalData.map.traffic_light.get_related_lights(tl_id)
            tl = TrafficLight(tl_id, 'traffic_light', stop_line, 'green', conflicted_lights, equal_lights)
            with self._lock_traffic_light_manage:
                self._traffic_light_pool[tl_id] = tl

        GlobalData.timer.reset()
        if self._use_record:
            self._recorder = Recorder()
        else:
            self._recorder = None

        if reload_render:
            if self._use_render:
                self._render_parent_pipe, self._render_child_pipe = multiprocessing.Pipe()
                self._render = SimRender(
                    data_pipe=self._render_parent_pipe,
                    map_file=os.path.join(GlobalData.map_root, GlobalData.map_name, 'map.json'),
                    host='0.0.0.0',
                    port=self._port,
                    debug=self._debug
                )

                self._render.run()
                logger.info('Start visualization render on port: localhost:{}'.format(self._port))
            else:
                self._render_parent_pipe = None
                self._render_child_pipe = None
                self._render = None

    def reload_world(
            self
    ):
        self._initialize(False)
        # TODO: update map reload

    ######### Timer Management #########
    @property
    def game_time(self):
        return GlobalData.timer.get_game_time()

    @property
    def real_time(self):
        return GlobalData.timer.get_real_time_elapsed()

    @property
    def frame_count(self):
        return GlobalData.timer.get_frame_count()

    @property
    def frequency(self):
        return float(GlobalData.sim_frequency)

    ######### Map/Tool Management #########
    @property
    def map(self):
        return GlobalData.map

    @property
    def map_name(self):
        return GlobalData.map_name

    ######### Actor management (including traffic lights) #########
    @property
    def actors(self):
        with self._lock_actor_manage:
            return self._actor_pool # maybe modified

    def register_actor(self, actor):
        if actor.id in self._actor_pool:
            raise KeyError(f"Actor with id {actor.id} already exists.")
        with self._lock_actor_manage:
            self._actor_pool[actor.id] = actor

    def remove_actor(self, actor_id):
        if actor_id not in self._actor_pool:
            logger.warning(f"Actor with id {actor_id} not exists.")
            return
        with self._lock_actor_manage:
            self._actor_pool.pop(actor_id)

    def get_actor(self, actor_id):
        if actor_id not in self._actor_pool:
            logger.warning(f"Actor with id {actor_id} not exists.")
            return None
        with self._lock_actor_manage:
            return self._actor_pool[actor_id]

    ######### Traffic Light Management #########
    @property
    def traffic_lights(self):
        with self._lock_traffic_light_manage:
            return self._traffic_light_pool # maybe modified

    ######### Oracle Management #########
    @property
    def termination(self):
        return self._termination

    def register_oracle(self, oracle):
        with self._lock_oracle_manage:
            self._oracle_pool[oracle.id] = oracle

    def remove_oracle(self, oracle_id):
        if oracle_id not in self._oracle_pool:
            logger.warning(f"Oracle with id {oracle_id} not exists.")
            return
        with self._lock_oracle_manage:
            self._oracle_pool.pop(oracle_id)

    def get_oracle(self, oracle_id):
        if oracle_id not in self._oracle_pool:
            logger.warning(f"Oracle with id {oracle_id} not exists.")
            return None
        with self._lock_oracle_manage:
            return self._oracle_pool[oracle_id] # maybe modified

    ######### Fitness Management #########
    def register_fitness(self, fitness):
        with self._lock_fitness_manage:
            self._fitness_pool[fitness.id] = fitness

    def remove_fitness(self, fitness_id):
        if fitness_id not in self._fitness_pool:
            logger.warning(f"Fitness with id {fitness_id} not exists.")
            return
        with self._lock_fitness_manage:
            self._fitness_pool.pop(fitness_id)

    def get_fitness(self, fitness_id):
        if fitness_id not in self._fitness_pool:
            logger.warning(f"Fitness with id {fitness_id} not exists.")
            return None
        with self._lock_fitness_manage:
            return self._fitness_pool[fitness_id]

    def rendering(self):
        # get record json data
        # only keep actor & traffic lights
        if (self._recorder is not None) or (self._render is not None):
            frame_data = {
                'map_name': GlobalData.map_name,
                'frame': GlobalData.timer.get_frame_count(),
                'game_time': GlobalData.timer.get_game_time(),
                'real_time': GlobalData.timer.get_real_time_elapsed(),
                'actors': [actor.json_data() for actor in self._actor_pool.values()],
                'traffic_lights': [tl.json_data() for tl in self._traffic_light_pool.values()]
            }
        else:
            frame_data = None

        if self._recorder is not None:
            self._recorder.update(frame_data)

        if self._render is not None:
            self._render_child_pipe.send(frame_data)

    ######### Running simulation #########
    def tick(self):
        if not self._render_first:
            self._render_first = True
            # self.rendering()

        # Thread count (up to 24 threads)
        max_threads = min(24, multiprocessing.cpu_count())
        delta_time = 1 / GlobalData.sim_frequency

        # Process actors and traffic lights
        actor_list = list(self._actor_pool.values()) + list(self._traffic_light_pool.values())
        oracle_list = list(self._oracle_pool.values())

        with ThreadPoolExecutor(max_threads) as executor:
            # Process actors
            executor.map(lambda actor: actor.tick(delta_time), actor_list)
            # Process oracles
            executor.map(lambda oracle: oracle.tick(delta_time), oracle_list)

        # Update termination
        self._termination = all([oracle.termination for oracle in self._oracle_pool.values()])

        # Update timer
        GlobalData.timer.tick()

        # Render
        self.rendering()

    def async_run(self):
        while not self.termination:
            self.tick()
            time.sleep(1 / GlobalData.sim_frequency) # this may be modified

    def start(self):
        # render & record the first step
        self._termination = False
        self._thread_run = Thread(target=self.async_run)
        GlobalData.timer.reset()
        self._thread_run.start()

    def stop(self):
        # this is used for cleaning
        self._termination = True
        if self._thread_run is not None:
            self._thread_run.join()
            self._thread_run = None

    def close(self):
        self.stop()
        if self._render is not None:
            self._render.close()

    ######### Saver Manager #########
    def export_record(self, filename: str):
        if self._recorder is not None:
            self._recorder.save(filename)
        else:
            logger.warning("No record data to save. Please turn on the record flag.")

    def export_result(self, filename: Optional[str] = None) -> Dict:
        oracle_result = {oracle.id: oracle.termination for oracle in self._oracle_pool.values()}
        fitness_result = {fitness.id: fitness.result for fitness in self._fitness_pool.values()}
        result = {
            'oracle': oracle_result,
            'fitness': fitness_result
        }

        if filename is not None:
            with open(filename, 'w') as f:
                json.dump(result, f, indent=4)
        return result
