from typing import Optional
from apollo_sim.tools.timer import Timer
from apollo_sim.map.map_loader import Map


class GlobalData:

    sim_frequency: float = 25.0

    map_root: Optional[str] = None
    map_name: Optional[str] = "borregas_ave"
    apollo_root: Optional[str] = "/apollo"

    map: Optional[Map] = None
    timer: Optional[Timer] = None