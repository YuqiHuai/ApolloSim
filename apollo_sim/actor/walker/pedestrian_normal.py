from apollo_sim.registry import ACTOR_REGISTRY
from apollo_sim.actor.walker.base import WalkerActor, BoundingBox, Location

@ACTOR_REGISTRY.register("walker.pedestrian.normal")
class PedestrianNormal(WalkerActor):

    category = "walker.pedestrian.normal"
    _bbox = BoundingBox(
        length=0.5,
        width=0.5,
        height=1.8,
    )

    _max_acceleration = 10.0
    _max_deceleration = 10.0

    _front_edge_to_center = 0.25
    _back_edge_to_center = 0.25
    _left_edge_to_center = 0.25
    _right_edge_to_center = 0.25

    def __init__(self, id: int, location: Location, role: str = 'walker'):
        super(PedestrianNormal, self).__init__(id, location, role)

