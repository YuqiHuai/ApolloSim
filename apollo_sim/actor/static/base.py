import copy
import math

from typing import List, Tuple
from shapely.geometry import Polygon, Point

from apollo_modules.modules.common.proto.geometry_pb2 import Point3D

from apollo_sim.actor.base import Actor
from apollo_sim.actor.misc import right_rotation
from apollo_sim.actor.basic import Location, BoundingBox

class StaticActor(Actor):

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

    # for vis or others
    _polygon: List = None

    def __init__(self, id: int, location: Location, role: str):
        super(StaticActor, self).__init__(id, role)
        self._location = copy.deepcopy(location)
        _, _, self._polygon = self.get_polygon(buffer=0.0)

    @property
    def speed(self) -> float:
        return 0.0

    ###### public properties ######
    @property
    def location(self) -> Location:
        return copy.deepcopy(self._location)

    @property
    def bbox(self) -> BoundingBox:
        return copy.deepcopy(self._bbox)

    def update_location(self, location: Location):
        with self._thread_lock:
            self._location = copy.deepcopy(location)

            # cal vis
            _, _, self._polygon = self.get_polygon(buffer=0.0)

    def get_forward_vector(self) -> List:
        init_vector = [1, 0]
        forward_vector = right_rotation(init_vector, -self._location.yaw)
        return forward_vector

    def get_polygon(self, buffer: float = 0.0) -> Tuple[Polygon, List[Point3D], List]:
        half_w = self._bbox.width / 2.0

        front_l = self._bbox.length / 2.0
        back_l = -1 * self._bbox.length / 2.0
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
            "role": self.role,
            "polygon": self._polygon
        }

    def _tick(self, delta_time: float):
        pass