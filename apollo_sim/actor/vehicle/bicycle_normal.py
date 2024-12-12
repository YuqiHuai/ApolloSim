from apollo_sim.registry import ACTOR_REGISTRY
from apollo_sim.actor.vehicle.base import VehicleActor, Location, BoundingBox

@ACTOR_REGISTRY.register("vehicle.bicycle.normal")
class BicycleNormal(VehicleActor):

    # basic information - fixed
    category: str = "vehicle.bicycle.normal"

    _bbox: BoundingBox = BoundingBox(
        length=3.0,
        width=1.0,
        height=1.8
    )

    _max_acceleration: float = 2.0 #5.59 # not accuracy
    _max_deceleration: float = -6.0

    _front_edge_to_center: float = 1.5
    _back_edge_to_center: float = 1.5
    _left_edge_to_center: float = 0.5
    _right_edge_to_center: float = 0.5

    _max_steer_angle: float = 8.20304748437 # radians * 180 / math.pi
    _steer_ratio: float = 16.0

    _wheelbase: float = 2.8448
    _max_abs_speed_when_stopped: float = 0.2

    def __init__(self, id: int, location: Location, role: str):
        super(BicycleNormal, self).__init__(id, location, role)
