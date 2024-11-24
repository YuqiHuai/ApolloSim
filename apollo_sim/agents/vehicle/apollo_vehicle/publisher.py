import time
import math
import numpy as np

from loguru import logger
from typing import Dict
from scipy.spatial.transform import Rotation

from apollo_modules.modules.common.proto.geometry_pb2 import Point3D, PointENU, Quaternion
from apollo_modules.modules.common.proto.header_pb2 import Header
from apollo_modules.modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from apollo_modules.modules.localization.proto.localization_pb2 import LocalizationEstimate
from apollo_modules.modules.localization.proto.pose_pb2 import Pose
from apollo_modules.modules.routing.proto.routing_pb2 import LaneWaypoint, RoutingRequest
from apollo_modules.modules.perception.proto.traffic_light_detection_pb2 import TrafficLight, TrafficLightDetection
from apollo_modules.modules.canbus.proto.chassis_pb2 import Chassis

from apollo_sim.actor import Waypoint, ActorClass
from apollo_sim.actor.signal.traffic_light import TrafficLightState
from apollo_sim.bridge.apollo.container import ApolloContainer
from apollo_sim.bridge.apollo.cyber_bridge import Topics

def inverse_quaternion_rotate(orientation: Quaternion, vector: np.ndarray) -> np.ndarray:
    # Create a quaternion from the given orientation (w, x, y, z)
    quaternion = Rotation.from_quat([orientation.qx, orientation.qy, orientation.qz, orientation.qw])

    # Get the inverse of the rotation matrix
    rotation_matrix_inv = quaternion.inv().as_matrix()

    # Apply the inverse rotation to the vector
    transformed_vector = rotation_matrix_inv.dot(vector)

    return transformed_vector


def transform_to_vrf(point_mrf: Point3D, orientation: Quaternion) -> Point3D:
    v_mrf = np.array([point_mrf.x, point_mrf.y, point_mrf.z])
    # Rotate the vector using the inverse of the quaternion
    v_vrf = inverse_quaternion_rotate(orientation, v_mrf)
    # Set the transformed coordinates in point_vrf
    return Point3D(
        x=v_vrf[0],
        y=v_vrf[1],
        z=v_vrf[2],
    )

class ApolloPublisher:
    """
    Class to manage and communicate with an Apollo instance/container
    """

    def __init__(
            self,
            idx: int,
            trigger_time: float,
            route: list[Waypoint],
            container_name: str = None,
            prefix: str = 'apollo',
    ):
        self.idx = idx
        self.trigger_time = trigger_time
        self.route = route
        self.prefix = prefix
        if container_name is not None:
            self.container_name = container_name
        else:
            self.container_name = f"{self.prefix}_{self.idx}"

        # other data
        self.routing_response = False
        self.control_data = None
        self.throttle_percentage = 0.0
        self.brake_percentage = 0.0
        self.steering_percentage = 0.0

        # inner parameters
        self.seq_num_chassis = 0
        self.seq_num_localization = 0
        self.seq_num_perception_obstacle = 0
        self.seq_num_traffic_light = 0
        self.seq_num_route = 0

        self.container = ApolloContainer(
            self.container_name
        )
        self.container.start_apollo()
        self.initialize_bridge()


    def initialize_bridge(self):
        """
        Resets and initializes all necessary modules of Apollo
        """
        self.register_publishers()
        self.register_subscribers()

        # make sure all connection before this step
        self.container.bridge.spin()
        logger.info('Initialized Apollo Wrapper Bridge')

    def stop(self):
        self.container.stop_bridge()
        self.container.stop_container()

    def register_publishers(self):
        """
        Register publishers for the cyberRT communication
        """
        for c in [Topics.Chassis, Topics.Localization, Topics.Obstacles, Topics.TrafficLight, Topics.RoutingRequest]:
            self.container.bridge.add_publisher(c)

    def register_subscribers(self):
        """
        Register subscribers for the cyberRT communication
        """

        def control_cb(data):
            """
            Callback function when control message is received
            publish new state to the traffic bridge
            """
            self.control_data = data
            self.throttle_percentage = data.throttle / 100.0
            self.brake_percentage = data.brake / 100.0
            self.steering_percentage = data.steering_target / 100.0

        self.container.bridge.add_subscriber(Topics.Control, control_cb)

    def publish_localization(self, actor: 'ActorClass'):
        """
        Should calculate the location & heading & speed
        require information:
            1. position v
            2. heading v
            3. orientation v
            4. linear_velocity v
            5. linear_acceleration v
            6. angular_velocity v
            7. linear_acceleration_vrf
            8. angular_acceleration_vrf

        :param actor:
        :return:
        """
        # convert state to apollo format
        # 1. position
        position = PointENU(x=actor.location.x, y=actor.location.y)
        # 2. heading
        heading = actor.location.yaw
        # 3. orientation
        # Adjust the heading as needed
        adjusted_heading = heading - (np.pi / 2)
        adjusted_heading = (adjusted_heading + math.pi) % (2 * math.pi) - math.pi
        # Create a rotation object from the adjusted heading
        rotation = Rotation.from_euler('z', adjusted_heading, degrees=False)
        # Extract quaternion components (x, y, z, w format)
        x, y, z, w = rotation.as_quat()
        orientation = Quaternion(
            qx=x, qy=y, qz=z, qw=w
        )
        # 4. linear_velocity
        linear_velocity_x = actor.speed * np.cos(heading)
        linear_velocity_y = actor.speed * np.sin(heading)
        linear_velocity = Point3D(
            x=linear_velocity_x, y=linear_velocity_y, z=0
        )
        # 5. linear_acceleration
        linear_acceleration_x = actor.acceleration * np.cos(heading)
        linear_acceleration_y = actor.acceleration * np.sin(heading)
        linear_acceleration = Point3D(
            x=linear_acceleration_x, y=linear_acceleration_y, z=0
        )
        # 6. angular_velocity
        angular_velocity_z = actor.angular_speed
        angular_velocity = Point3D(
            x=0.0, y=0.0, z=angular_velocity_z
        )

        # 7. linear_acceleration_vrf
        linear_acceleration_vrf = transform_to_vrf(linear_acceleration, orientation)

        # 8. angular_acceleration_vrf
        angular_velocity_vrf = transform_to_vrf(angular_velocity, orientation)

        loc = LocalizationEstimate(
            header=Header(
                timestamp_sec=time.time(),
                module_name="MAGGIE",
                sequence_num=self.seq_num_localization
            ),
            pose=Pose(
                # 1. position
                position=position,
                # 2. heading
                heading=heading,
                # 3. orientation
                orientation=orientation,
                # 4. linear_velocity
                linear_velocity=linear_velocity,
                # 5. linear_acceleration
                linear_acceleration=linear_acceleration,
                # 6. angular_velocity
                angular_velocity=angular_velocity,
                # 7. linear_acceleration_vrf
                linear_acceleration_vrf=linear_acceleration_vrf,
                # 8. angular_acceleration_vrf
                angular_velocity_vrf=angular_velocity_vrf
            )
        )

        self.container.bridge.publish(Topics.Localization, loc.SerializeToString())
        self.seq_num_localization += 1

    def publish_chassis(self, actor: 'ActorClass'):
        # TODO: check again this function
        header = Header(
            timestamp_sec=time.time(),
            module_name='MAGGIE',
            sequence_num=self.seq_num_chassis
        )

        speed_mps = actor.speed
        if self.control_data is not None:
            gear_location = self.control_data.gear_location
            if gear_location == Chassis.GearPosition.GEAR_REVERSE:
                speed_mps = -speed_mps
        else:
            gear_location = Chassis.GearPosition.GEAR_NEUTRAL

        chassis = Chassis(
            header=header,
            engine_started=True,
            driving_mode=Chassis.DrivingMode.COMPLETE_AUTO_DRIVE,
            gear_location=gear_location,
            speed_mps=speed_mps,
            throttle_percentage=actor.control.throttle * 100.0, # todo: this can be updated?
            brake_percentage=actor.control.brake * 100.0,
            steering_percentage=actor.control.steering * 100.0
        )

        self.container.bridge.publish(Topics.Chassis, chassis.SerializeToString())
        self.seq_num_chassis += 1

    def publish_obstacles(self, perception_obstacles: Dict[str, 'ActorClass']):
        apollo_perception = list()
        for obs_id, obs_actor in perception_obstacles.items():
            if obs_id == self.idx:
                continue
            loc = PointENU(x=obs_actor.location.x, y=obs_actor.location.y)
            position = Point3D(x=loc.x, y=loc.y, z=loc.z)
            velocity = Point3D(
                x=math.cos(obs_actor.location.yaw) * obs_actor.speed,
                y=math.sin(obs_actor.location.yaw) * obs_actor.speed,
                z=0.0
            )

            obs_polygon, apollo_points, _ = obs_actor.get_polygon()
            if obs_actor.category.split('.')[0] == 'vehicle':
                if obs_actor.category.split('.')[1] == 'bicycle':
                    obs_type = PerceptionObstacle.BICYCLE
                else:
                    obs_type = PerceptionObstacle.VEHICLE
            elif obs_actor.category.split('.')[0] == 'walker':
                obs_type = PerceptionObstacle.PEDESTRIAN
            elif obs_actor.category.split('.')[0] == 'static':
                obs_type = PerceptionObstacle.UNKNOWN_UNMOVABLE
            else:
                obs_type = PerceptionObstacle.UNKNOWN

            obs = PerceptionObstacle(
                id=obs_actor.id,
                position=position,
                theta=obs_actor.location.yaw,
                velocity=velocity,
                acceleration=Point3D(x=0, y=0, z=0),
                length=obs_actor.bbox.length,
                width=obs_actor.bbox.width,
                height=obs_actor.bbox.height,
                type=obs_type,
                timestamp=time.time(),
                tracking_time=1.0,
                polygon_point=apollo_points
            )
            apollo_perception.append(obs)

        header = Header(
            timestamp_sec=time.time(),
            module_name='MAGGIE',
            sequence_num=self.seq_num_perception_obstacle
        )
        perception_obstacles_bag = PerceptionObstacles(
            header=header,
            perception_obstacle=apollo_perception,
        )
        self.container.bridge.publish(Topics.Obstacles, perception_obstacles_bag.SerializeToString())
        self.seq_num_perception_obstacle += 1

    def publish_traffic_light(self, traffic_light_config: dict):
        # TODO: Why not stable publish to the apollo?
        tld = TrafficLightDetection()
        tld.header.timestamp_sec = time.time()
        tld.header.module_name = "MAGGIE"  # "MAGGIE"
        tld.header.sequence_num = self.seq_num_traffic_light
        for k in traffic_light_config.keys():
            traffic_light_state = traffic_light_config[k].get_state()
            tl = tld.traffic_light.add()
            tl.id = k
            tl.confidence = 1
            if traffic_light_state == TrafficLightState.green:
                tl.color = TrafficLight.Color.GREEN
            elif traffic_light_state == TrafficLightState.yellow:
                tl.color = TrafficLight.Color.YELLOW
            elif traffic_light_state == TrafficLightState.red:
                tl.color = TrafficLight.Color.RED
            elif traffic_light_state == TrafficLightState.unknown:
                tl.color = TrafficLight.Color.BLACK
            else:
                tl.color = TrafficLight.Color.UNKNOWN

        self.container.bridge.publish(Topics.TrafficLight, tld.SerializeToString())
        self.seq_num_traffic_light += 1

    def publish_route(self, t: float):
        """
        Send the instance's routing request to cyberRT
        """
        if t < self.trigger_time or self.routing_response:
            return

        heading = self.route[0].location.heading
        coord = PointENU(x=self.route[0].location.x, y=self.route[0].location.y)

        rr = RoutingRequest(
            header=Header(
                timestamp_sec=time.time(),
                module_name="MAGGIE",
                sequence_num=self.seq_num_route
            ),
            waypoint=[
                LaneWaypoint(
                    pose=coord,
                    heading=heading
                )
            ] + [
                LaneWaypoint(
                    id=x.lane.id,
                    s=x.lane.s,
                ) for x in self.route
            ]
        )

        self.container.bridge.publish(
            Topics.RoutingRequest, rr.SerializeToString()
        )

        self.routing_response = True
        self.seq_num_route += 1

    ######### Other Tools #########
    def recorder_operator(self, operation, record_folder=None, scenario_id=None):
        if operation == 'start':
            self.container.start_recorder(record_folder, scenario_id)
        elif operation == 'stop':
            self.container.stop_recorder()
        else:
            raise RuntimeError(f"Not supported operation: {operation}")

    def move_recording(self, record_folder: str, scenario_id: str, local_folder: str, delete: bool = True):
        self.container.copy_record(
            record_folder=record_folder,
            record_id=scenario_id,
            target_folder=local_folder,
            delete=delete
        )