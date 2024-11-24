from apollo_sim.actor import register_agent
from apollo_sim.actor.vehicle.base import VehicleActor, Location, BoundingBox

# https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/calibration/data/mkz_example/vehicle_param.pb.txt
@register_agent("vehicle.lincoln.mkz")
class LincolnMKZ(VehicleActor):

    # basic information - fixed
    category: str = 'vehicle.lincoln.mkz'

    _bbox: BoundingBox = BoundingBox(
        length=4.933,
        width=2.11,
        height=1.48
    )

    _max_acceleration: float = 2.0 #5.59 # not accuracy
    _max_deceleration: float = -6.0

    _front_edge_to_center: float = 3.89
    _back_edge_to_center: float = 1.043
    _left_edge_to_center: float = 1.055
    _right_edge_to_center: float = 1.055

    _max_steer_angle: float = 8.20304748437 # radians * 180 / math.pi
    _steer_ratio: float = 16.0

    _wheelbase: float = 2.8448
    _max_abs_speed_when_stopped: float = 0.2

    def __init__(self, id: int, location: Location, role: str):
        super(LincolnMKZ, self).__init__(id, location, role)
