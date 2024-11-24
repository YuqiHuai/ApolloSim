import os
import importlib

from apollo_sim.fitness.base import Fitness, FitnessClass

fitness_library = {}

def register_fitness(name):
    def decorator(cls):
        fitness_library[name] = cls
        return cls
    return decorator

# Dynamically import all modules in the agents folder
def discover_fitness(package_name="apollo_sim.fitness"):
    package_dir = os.path.dirname(__file__)
    for module_name in os.listdir(package_dir):
        if module_name.endswith(".py") and module_name != "__init__.py" and module_name != "base.py":
            module_name = module_name[:-3]  # Strip .py extension
            importlib.import_module(f"{package_name}.{module_name}")

# Discover and register agents when the package is imported
discover_fitness()