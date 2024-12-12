import os

from apollo_sim.registry.utils import discover_modules

discover_modules(os.path.dirname(__file__), package_name="apollo_sim")

__version__ = "7.0.0"  # Replace with your actual version
