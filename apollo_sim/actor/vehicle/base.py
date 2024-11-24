import copy
import math

from typing import List, Tuple
from shapely.geometry import Polygon, Point

from apollo_modules.modules.common.proto.geometry_pb2 import Point3D

from apollo_sim.actor.base import Actor
from apollo_sim.actor.basic import Location, BoundingBox
from apollo_sim.actor.misc import right_rotation, normalize_angle
from apollo_sim.actor.control import VehicleControl

class VehicleActor(Actor):

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
    _control: VehicleControl = VehicleControl(
        throttle=0.0,
        brake=0.0,
        steering=0.0,
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
    _max_steer_angle: float = 0.0  # radians * 180 / math.pi
    _steer_ratio: float = 0.0
    _wheelbase: float = 0.0
    _max_abs_speed_when_stopped: float = 0.0

    def __init__(self, id: int, location: Location, role: str):
        super(VehicleActor, self).__init__(id, role)
        self._location = copy.deepcopy(location)
        _, _, self._polygon = self.get_polygon(buffer=0.0)
        self._control = VehicleControl(
            throttle=0.0,
            brake=0.0,
            steering=0.0,
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
    def control(self) -> VehicleControl:
        return copy.deepcopy(self._control)

    def apply_control(self, control: VehicleControl or None):
        # write operation
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
        """
        All in range of [0, 1]
        NOW WE ONLY support linear model for each object
        """
        throttle = self._control.throttle
        brake = self._control.brake
        steering = self._control.steering

        assert 0 <= throttle <= 1
        assert 0 <= brake <= 1
        assert -1 <= steering <= 1

        self._last_location = copy.deepcopy(self._location)

        # 1. Compute current acceleration based on throttle and brake
        if throttle > 0.0 and brake == 0:
            # Accelerating based on throttle
            curr_acceleration = throttle * abs(self._max_acceleration)
        elif throttle == 0.0 and brake > 0:
            # Decelerating based on brake
            curr_acceleration = -brake * abs(self._max_deceleration)
        elif throttle == 0.0 and brake == 0:
            curr_acceleration = 0.0
        else:
            curr_acceleration = 0.0

        # 2. Update speed and ensure it doesn't go below zero
        curr_speed = self._speed
        if curr_speed <= 0.0:
            curr_acceleration = max(0.0, curr_acceleration)
        next_speed = curr_speed + curr_acceleration * delta_time # according to the frequency
        next_speed = max(0.0, next_speed)  # Ensure speed is non-negative

        # 3. Compute the steering angle in radians and angular velocity
        steering_angle = math.radians(steering * (self._max_steer_angle * 180 / math.pi) / self._steer_ratio)  # degree?
        if abs(steering_angle) > 1e-4:  # Avoid near-zero angle issues
            avg_speed = (curr_speed + next_speed) / 2.0  # Use average speed
            curr_angular_speed = avg_speed * math.tan(steering_angle) / self._wheelbase
        else:
            curr_angular_speed = 0.0

        # 4. Update position using the next speed
        next_x = self._location.x + next_speed * math.cos(self._location.yaw) * delta_time
        next_y = self._location.y + next_speed * math.sin(self._location.yaw) * delta_time

        # 5. Update heading and normalize it
        next_heading = normalize_angle(self._location.yaw + curr_angular_speed * delta_time)

        # 6. Create the next state
        self._location.x = next_x
        self._location.y = next_y
        self._location.yaw = next_heading
        self._speed = next_speed
        self._acceleration = curr_acceleration
        self._angular_speed = curr_angular_speed

        # cal vis
        _, _, self._polygon = self.get_polygon(buffer=0.0)