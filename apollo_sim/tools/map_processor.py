import os.path
import time

from loguru import logger
from apollo_sim.map.map_loader import Map

def preprocess_apollo_map(map_name: str, apollo_dir: str, map_dir: str):
    """
    Preprocess Apollo map
    """
    logger.info(f'Preprocessing Apollo map: {map_name}')
    start_time = time.time()
    map_instance = Map()
    map_instance.parse_from_source(os.path.join(apollo_dir, map_name, 'base_map.bin'))
    map_instance.export(os.path.join(map_dir, map_name))
    logger.info(f'Preprocessing Apollo map: {map_name} finished in {time.time() - start_time:.2f}s')