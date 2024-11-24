import copy
import math
import numpy as np

from loguru import logger
from typing import List, Tuple
from shapely.geometry import Polygon, Point

from apollo_modules.modules.common.proto.geometry_pb2 import Point3D

from apollo_sim.actor.base import Actor
from apollo_sim.actor.basic import Location, BoundingBox
from apollo_sim.actor.misc import right_rotation, normalize_angle
from apollo_sim.actor.control import WalkerControl

class WalkerActor(Actor):

    _bbox: BoundingBox = BoundingBox(
        length=0.0,
        width=0.0,
        height=0.0
    )
    _location: Location = Location(
        x=0.0,
        y=0.0,
        z=0.0,
        pitch=0.0,
        yaw=0.0,
        roll=0.0,
    )
    _speed: float = 0.0
    _angular_speed: float = 0.0
    _acceleration: float = 0.0
    _control: WalkerControl = WalkerControl(
        acceleration=0.0,
        heading=0.0
    )
    _last_location: Location = Location(
        x=0.0,
        y=0.0,
        z=0.0,
        pitch=0.0,
        yaw=0.0,
        roll=0.0,
    )

    # for vis or others
    _polygon: List = None

    # some other attributes
    _max_acceleration: float = 0.0
    _max_deceleration: float = 0.0
    _front_edge_to_center: float = 0.0
    _back_edge_to_center: float = 0.0
    _left_edge_to_center: float = 0.0
    _right_edge_to_center: float = 0.0

    def __init__(self, id: int, location: Location, role: str):
        super(WalkerActor, self).__init__(id, role)
        self._location = copy.deepcopy(location)
        _, _, self._polygon = self.get_polygon(buffer=0.0)
        self._control = WalkerControl(
            acceleration=0.0,
            heading=0.0
        )

    ###### public properties ######
    @property
    def location(self) -> Location:
        return copy.deepcopy(self._location)

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def angular_speed(self) -> float:
        return self._angular_speed

    @property
    def acceleration(self) -> float:
        return self._acceleration

    @property
    def bbox(self) -> BoundingBox:
        return copy.deepcopy(self._bbox)

    @property
    def control(self) -> WalkerControl:
        with self._thread_lock:
            return copy.deepcopy(self._control)

    def apply_control(self, control: WalkerControl or None):
        with self._thread_lock:
            self._control = copy.deepcopy(control)

    def get_forward_vector(self) -> List:
        init_vector = [1, 0]
        forward_vector = right_rotation(init_vector, -self._location.yaw)
        return forward_vector

    def get_polygon(self, buffer: float = 0.0) -> Tuple[Polygon, List[Point3D], List]:
        half_w = self._bbox.width / 2.0

        front_l = self._bbox.length - self._back_edge_to_center
        back_l = -1 * self._back_edge_to_center
        front_l += buffer

        sin_h = math.sin(self._location.yaw)
        cos_h = math.cos(self._location.yaw)
        vectors = [(front_l * cos_h - half_w * sin_h,
                    front_l * sin_h + half_w * cos_h),
                   (back_l * cos_h - half_w * sin_h,
                    back_l * sin_h + half_w * cos_h),
                   (back_l * cos_h + half_w * sin_h,
                    back_l * sin_h - half_w * cos_h),
                   (front_l * cos_h + half_w * sin_h,
                    front_l * sin_h - half_w * cos_h)]

        points = []
        apollo_points = []  # Apollo Points
        for x, y in vectors:
            points.append([self._location.x + x, self._location.y + y])
            p = Point3D()
            p.x = self._location.x + x
            p.y = self._location.y + y
            p.z = 0.0
            apollo_points.append(p)

        return Polygon(points), apollo_points, points

    def dist2actor(self, agent) -> float:
        self_polygon, _, _ = self.get_polygon(buffer=0.0)
        agent_polygon, _, _ = agent.get_polygon(buffer=0.0)
        return self_polygon.distance(agent_polygon)

    def dist2point(self, point: Point):
        self_polygon, _, _ = self.get_polygon()
        return self_polygon.distance(point)

    def dist2polygon(self, polygon: Polygon):
        self_polygon, _, _ = self.get_polygon()
        return self_polygon.distance(polygon)

    def _json_data(self):
        return {
            "id": self.id,
            "category": self.category,
            "bbox": self._bbox.json_data(),
            "location": self._location.json_data(),
            "speed": self._speed,
            "angular_speed": self._angular_speed,
            "acceleration": self._acceleration,
            "control": self._control.json_data(),
            "role": self.role,
            "last_location": self._last_location.json_data(),
            "polygon": self._polygon
        }

    def _tick(self, delta_time: float):
        acceleration = self._control.acceleration
        heading = self._control.heading

        self._last_location = copy.deepcopy(self._location)
        curr_acceleration = float(np.clip(acceleration, -abs(self._max_deceleration), abs(self._max_acceleration)))
        curr_speed = self._speed
        next_speed = curr_speed + curr_acceleration * delta_time  # according to the frequency
        next_speed = max(0.0, next_speed)  # Ensure speed is non-negative

        next_heading = normalize_angle(heading)

        next_x = self._location.x + next_speed * math.cos(next_heading) * delta_time
        next_y = self._location.y + next_speed * math.sin(next_heading) * delta_time

        # 6. Create the next state
        self._location.x = next_x
        self._location.y = next_y
        self._location.yaw = next_heading
        self._speed = next_speed
        self._acceleration = curr_acceleration
        self._angular_speed = next_speed

        # cal vis
        _, _, self._polygon = self.get_polygon(buffer=0.0)

