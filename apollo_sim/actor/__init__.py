import os
import importlib
import fnmatch

from apollo_sim.actor.basic import *
from apollo_sim.actor.base import ActorClass

actor_library = {}

def register_agent(name):
    def decorator(cls):
        actor_library[name] = cls
        return cls
    return decorator

def discover_agents(package_name="apollo_sim.actor"):
    package_dir = os.path.dirname(__file__)
    for root, _, files in os.walk(package_dir):
        for file in files:
            if file.endswith(".py") and file != "__init__.py" and file != "base.py" and file != "misc.py":
                # Create a module path from the folder structure
                rel_dir = os.path.relpath(root, package_dir)
                module_name = file[:-3]  # Strip .py extension
                full_module_name = f"{package_name}.{rel_dir.replace(os.sep, '.')}.{module_name}"
                importlib.import_module(full_module_name)

# Function to filter actors by prefix
def filter_actors(pattern):
    """
    Filter actors in the actor library based on a given pattern.

    Args:
        pattern (str): The pattern to match (e.g., "vehicle*").

    Returns:
        dict: A dictionary of actor names and their corresponding classes matching the pattern.
    """
    return {name: cls for name, cls in actor_library.items() if fnmatch.fnmatch(name, pattern)}

discover_agents()
