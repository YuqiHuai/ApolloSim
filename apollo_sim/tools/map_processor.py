from apollo_sim.map.apollo_map import ApolloMapLoader
from apollo_sim import map_root, apollo_root

def preprocess_apollo_map(map_name: str, apollo_dir: str, map_dir: str):
    # set root: TODO: update this
    apollo_root = apollo_dir
    map_root = map_dir
    map_loader = ApolloMapLoader()
    map_loader.convert_map(map_name)