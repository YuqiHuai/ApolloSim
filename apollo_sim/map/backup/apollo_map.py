"""
This part is modified by refering to the implementation of the MapParser class in DoppelTest:
Code source: YuqiHuai
Source URL: https://github.com/Software-Aurora-Lab/DoppelTest/blob/main/hdmap/MapParser.py
License: GNU General Public License v3.0, https://github.com/Software-Aurora-Lab/DoppelTest/blob/main/LICENSE
"""
import os
import copy
import math
import pickle
import random
import networkx as nx

from loguru import logger
from rtree.index import Index
from collections import defaultdict
from typing import List, Set, Tuple, Any
from shapely.geometry import LineString, Point, Polygon

from apollo_modules.modules.map.proto.map_crosswalk_pb2 import Crosswalk
from apollo_modules.modules.map.proto.map_junction_pb2 import Junction
from apollo_modules.modules.map.proto.map_lane_pb2 import Lane
from apollo_modules.modules.map.proto.map_pb2 import Map
from apollo_modules.modules.map.proto.map_signal_pb2 import Signal
from apollo_modules.modules.map.proto.map_stop_sign_pb2 import StopSign
from apollo_modules.modules.common.proto.geometry_pb2 import PointENU

from apollo_sim import map_root, apollo_root
from apollo_sim.actor import Location
from apollo_sim.actor.basic.waypoint import Waypoint

def load_hd_map(filename: str):
    apollo_map = Map()
    f = open(filename, 'rb')
    apollo_map.ParseFromString(f.read())
    f.close()
    return apollo_map

# this is the class that will be used to load the map from Apollo format
class ApolloMap(object):

    __map: Map
    __junctions: dict
    __signals: dict
    __stop_signs: dict
    __lanes: dict
    __crosswalk: dict
    __overlaps: dict

    __signals_at_junction: dict
    __lanes_at_junction: dict
    __lanes_controlled_by_signal: dict

    __signal_relations: nx.Graph
    __lane_nx: nx.DiGraph
    __lane_index: Index

    def __init__(self):
        pass

    def save_to_pkl(self, filename):
        # Gather all instance variables, remove name mangling
        attributes_dict = {key.split('__')[-1]: value for key, value in self.__dict__.items()}

        # Save the dictionary to a .pkl file
        with open(filename, 'wb') as f:
            pickle.dump(attributes_dict, f)

        logger.info(f"Attributes saved to {filename}")

    def load_from_pkl(self, map_name):
        file_path = os.path.join(map_root, map_name, 'map.pickle')
        # Load the dictionary from a .pkl file and update the instance variables
        with open(file_path, 'rb') as f:
            attributes_dict = pickle.load(f)

        # Restore the attributes with the original names
        for key, value in attributes_dict.items():
            setattr(self, f"_{self.__class__.__name__}__{key}", value)

        logger.info(f"Attributes loaded from {file_path}")

    def convert_map(
            self,
            map_name: str,
            # source_map_file: str,
            # save_map_file: str
    ):
        map_file = os.path.join(apollo_root, 'modules/map/data', map_name, 'base_map.bin')
        logger.info(f'Load map from {map_file}')

        map_pickle = os.path.join(map_root, map_name, 'map.pickle')
        # map_pickle = save_map_file
        map_folder = os.path.dirname(map_pickle)
        if not os.path.exists(map_folder):
            os.makedirs(map_folder)

        logger.info(f'Converting and saving map to {map_pickle}')
        self.__map = load_hd_map(map_file)
        self.load_junctions()
        self.load_signals()
        self.load_stop_signs()
        self.load_lanes()
        self.load_crosswalks()
        self.load_overlaps()
        self.parse_relations()
        self.parse_signal_relations()
        self.parse_lane_relations()
        self.build_spatial_index()

        self.save_to_pkl(map_pickle)

    def __is_overlap(self, obj1, obj2):
        """
        Check if 2 objects (e.g., lanes, junctions) have overlap

        :param any obj1: left hand side
        :param any obj2: right hand side
        """
        oid1 = set([x.id for x in obj1.overlap_id])
        oid2 = set([x.id for x in obj2.overlap_id])
        return oid1 & oid2 != set()

    ######## Load tools ########
    def load_junctions(self):
        """
        Load junctions on the HD Map
        """
        self.__junctions = dict()
        for junc in self.__map.junction:
            self.__junctions[junc.id.id] = junc

    def load_signals(self):
        """
        Load traffic signals on the HD Map
        """
        self.__signals = dict()
        for sig in self.__map.signal:
            self.__signals[sig.id.id] = sig

    def load_stop_signs(self):
        """
        Load stop signs on the HD Map
        """
        self.__stop_signs = dict()
        for ss in self.__map.stop_sign:
            self.__stop_signs[ss.id.id] = ss

    def load_lanes(self):
        """
        Load lanes on the HD Map
        """
        self.__lanes = dict()
        for l in self.__map.lane:
            self.__lanes[l.id.id] = l

    def load_crosswalks(self):
        """
        Load crosswalks on the HD Map
        """
        self.__crosswalk = dict()
        for cw in self.__map.crosswalk:
            self.__crosswalk[cw.id.id] = cw

    def load_overlaps(self):
        """
        Load lanes on the HD Map
        """
        self.__overlaps = dict()
        for o in self.__map.overlap:
            self.__overlaps[o.id.id] = o

    def parse_relations(self):
        """
        Parse relations between signals and junctions,
        lanes and junctions, and lanes and signals
        """
        # load signals at junction
        self.__signals_at_junction = defaultdict(list)
        for sigk, sigv in self.__signals.items():
            for junk, junv in self.__junctions.items():
                if self.__is_overlap(sigv, junv):
                    self.__signals_at_junction[junk].append(sigk)

        # load lanes at junction
        self.__lanes_at_junction = defaultdict(list)
        for lank, lanv in self.__lanes.items():
            for junk, junv in self.__junctions.items():
                if self.__is_overlap(lanv, junv):
                    self.__lanes_at_junction[junk].append(lank)

        # load lanes controlled by signal
        self.__lanes_controlled_by_signal = defaultdict(list)
        for junk, junv in self.__junctions.items():
            signal_ids = self.__signals_at_junction[junk]
            lane_ids = self.__lanes_at_junction[junk]
            for sid in signal_ids:
                for lid in lane_ids:
                    if self.__is_overlap(self.__signals[sid], self.__lanes[lid]):
                        self.__lanes_controlled_by_signal[sid].append(lid)

    def parse_signal_relations(self):
        """
        Analyze the relation between signals (e.g., signals that
        cannot be green at the same time)
        """
        g = nx.Graph()
        for junk, junv in self.__junctions.items():
            signal_ids = self.__signals_at_junction[junk]
            for sid1 in signal_ids:
                g.add_node(sid1)
                for sid2 in signal_ids:
                    if sid1 == sid2:
                        continue
                    lg1 = self.__lanes_controlled_by_signal[sid1]
                    lg2 = self.__lanes_controlled_by_signal[sid2]
                    if lg1 == lg2:
                        g.add_edge(sid1, sid2, v='EQ')
                    elif self.is_conflict_lanes(lg1, lg2):
                        g.add_edge(sid1, sid2, v='NE')
        self.__signal_relations = g

    def parse_lane_relations(self):
        """
        Analyze the relation between lanes (e.g., which lane is connected
        to which lane)

        :note: the relation is supposed to be included in the HD Map
          via predecessor and successor relation, but experimentally
          we found HD Map may be buggy and leave out some information,
          causing Routing module to fail.
        """
        dg = nx.DiGraph()
        for lane1 in self.__lanes:
            dg.add_node(lane1)
            for lane2 in self.__lanes:
                if lane1 == lane2:
                    continue
                line1 = self.get_lane_central_curve(lane1)
                line2 = self.get_lane_central_curve(lane2)
                s1, e1 = Point(line1.coords[0]), Point(line1.coords[-1])
                s2, e2 = Point(line2.coords[0]), Point(line2.coords[-1])

                if s1.distance(e2) < 0.001:
                    dg.add_edge(lane2, lane1)
                elif e1.distance(s2) < 0.001:
                    dg.add_edge(lane1, lane2)
        self.__lane_nx = dg

    def build_spatial_index(self):
        self.__lane_index = Index()
        for lane_id, lane_obj in self.__lanes.items():
            lane_polygon = self.get_lane_polygon(lane_id)
            minx, miny, maxx, maxy = lane_polygon.bounds
            self.__lane_index.insert(lane_id, (minx, miny, maxx, maxy))  # Insert using bounding box

    ######### properties - all return ids #########
    @property
    def junctions(self) -> List[str]:
        """
        Get a list of all junction IDs on the HD Map

        :returns: list of junction IDs
        :rtype: List[str]
        """
        return list(self.__junctions.keys())

    @property
    def lanes(self) -> List[str]:
        """
        Get a list of all lane IDs on the HD Map

        :returns: list of lane IDs
        :rtype: List[str]
        """
        return list(self.__lanes.keys())

    @property
    def crosswalks(self) -> List[str]:
        """
        Get a list of all crosswalk IDs on the HD Map

        :returns: list of crosswalk IDs
        :rtype: List[str]
        """
        return list(self.__crosswalk.keys())

    @property
    def signals(self) -> List[str]:
        """
        Get a list of all signal IDs on the HD Map

        :returns: list of signal IDs
        :rtype: List[str]
        """
        return list(self.__signals.keys())

    @property
    def stop_signs(self) -> List[str]:
        """
        Get a list of all stop sign IDs on the HD Map

        :returns: list of stop sign IDs
        :rtype: List[str]
        """
        return list(self.__stop_signs.keys())

    @property
    def overlaps(self) -> List[str]:
        """
        Get a list of all overlap IDs on the HD Map

        :returns: list of overlap IDs
        :rtype: List[str]
        """
        return list(self.__overlaps.keys())

    ######### determination #########
    def is_conflict_lanes(self, lane_id1: List[str], lane_id2: List[str]) -> bool:
        """
        Check if 2 groups of lanes intersect with each other

        :param List[str] lane_id1: list of lane ids
        :param List[str] lane_id2: another list of lane ids

        :returns: True if at least 1 lane from lhs intersects with another from rhs,
            False otherwise.
        :rtype: bool
        """
        for lid1 in lane_id1:
            for lid2 in lane_id2:
                if lid1 == lid2:
                    continue
                lane1 = self.get_lane_central_curve(lid1)
                lane2 = self.get_lane_central_curve(lid2)
                if lane1.intersects(lane2):
                    return True
        return False

    def is_junction_lane(self, lane_id: str) -> bool:
        """
        Return the junction id where the lane in the junction
        """
        for k, v in self.__lanes_at_junction.items():
            if lane_id in v:
                return True
        return False

    def is_driving_lane(self, lane_id: str) -> bool:
        lane_obj = self.get_lane(lane_id)
        lane_type = lane_obj.type
        if lane_type == lane_obj.LaneType.CITY_DRIVING:
            return True
        else:
            return False

    ######### getter #########
    def get_signals_wrt(self, signal_id: str) -> List[Tuple[str, str]]:
        """
        Get signals that have constraint with the specified signal

        :param str signal_id: ID of the signal interested in

        :returns: list of tuple each indicates the signal and the constraint
        :rtype: List[Tuple[str, str]]

        :example: ``[('signal_5', 'EQ'), ('signal_6', 'NE')]``
          indicates ``signal_5`` should have the same color, ``signal_6``
          cannot be green if the signal passed in is green.
        """
        result = list()
        for u, v, data in self.__signal_relations.edges(signal_id, data=True):
            result.append((v, data['v']))
        return result

    ######### getter - lane #########
    def get_lane(self, l_id: str) -> Lane:
        """
        Get a specific junction object based on ID

        :param str l_id: ID of the lane interested in

        :returns: lane object
        :rtype: Lane
        """
        return self.__lanes[l_id]

    def get_lane_central_curve(self, lane_id: str) -> LineString:
        """
        Gets the central curve of the lane.

        :param str lane_id: ID of the lane interested in

        :returns: an object representing the lane's central curve
        :rypte: LineString
        """
        lane = self.__lanes[lane_id]
        points = lane.central_curve.segment[0].line_segment
        line = LineString([[x.x, x.y] for x in points.point])
        return line

    def get_lane_boundary_curve(self, lane_id: str) -> Tuple[LineString, LineString]:
        lane = self.__lanes[lane_id]
        points = lane.left_boundary.curve.segment[0].line_segment
        left_line = LineString([[x.x, x.y] for x in points.point])
        points = lane.right_boundary.curve.segment[0].line_segment
        right_line = LineString([[x.x, x.y] for x in points.point])
        return left_line, right_line

    def get_lane_polygon(self, lane_id: str) -> Polygon:
        lane = self.__lanes[lane_id]
        points = lane.left_boundary.curve.segment[0].line_segment
        left_line = [[x.x, x.y] for x in points.point]
        points = lane.right_boundary.curve.segment[0].line_segment
        right_line = [[x.x, x.y] for x in points.point]

        right_line = right_line[::-1]
        lane_boundary = left_line + right_line
        return Polygon(lane_boundary)

    def get_lane_width(self, lane_id: str) -> float:
        return self.__lanes[lane_id].width

    def get_lane_length(self, lane_id: str) -> float:
        """
        Gets the length of the lane.

        :param str lane_id: ID of the lane interested in

        :returns: length of the lane
        :rtype: float
        """
        return self.get_lane_central_curve(lane_id).length

    def get_lanes_not_in_junction(self) -> Set[str]:
        """
        Get the set of all lanes that are not in the junction.

        :returns: ID of lanes who is not in a junction
        :rtype: Set[str]
        """
        lanes = set(self.lanes)
        for junc in self.__lanes_at_junction:
            jlanes = set(self.__lanes_at_junction[junc])
            lanes = lanes - jlanes
        return lanes

    def get_lanes_in_junction(self, junc_id: str) -> List[str]:
        return list(set(self.__lanes_at_junction[junc_id]))

    def get_reachable_lanes(self, lane_id: str, depth: int = 8) -> List[List[str]]:
        """
        Recursive method to compute paths no more than ``depth`` lanes from
        the starting lane.

        :param str lane_id: ID of the starting lane
        :param int depth: maximum number of lanes traveled

        :returns: list of possible paths
        :rtype: List[List[str]]
        """
        if depth == 1:
            return [[lane_id]]
        result = list()
        for u, v in self.__lane_nx.edges(lane_id):
            result.append([u, v])
            for rp in self.get_reachable_lanes(v, depth - 1):
                result.append([u] + rp)
        return result

    def get_predecessor_lanes(self, lane_id: str, driving_only=False) -> List[str]:
        current_lane = self.__lanes[lane_id]
        predecessor_lane_ids = current_lane.predecessor_id
        predecessor_lane_lst = []
        for item in predecessor_lane_ids:
            predecessor_lane_id = item.id
            if driving_only:
                if self.is_driving_lane(predecessor_lane_id):
                    predecessor_lane_lst.append(predecessor_lane_id)
            else:
                predecessor_lane_lst.append(predecessor_lane_id)
        return predecessor_lane_lst

    def get_successor_lanes(self, lane_id: str, driving_only=False) -> List[str]:
        current_lane = self.__lanes[lane_id]
        successor_lane_ids = current_lane.successor_id
        successor_lane_lst = []
        for item in successor_lane_ids:
            successor_lane_id = item.id
            if driving_only:
                if self.is_driving_lane(successor_lane_id):
                    successor_lane_lst.append(successor_lane_id)
            else:
                successor_lane_lst.append(successor_lane_id)
        return successor_lane_lst

    def get_left_neighbor_forward_lanes(self, lane_id: str, driving_only=False) -> List[str]:
        current_lane = self.__lanes[lane_id]
        left_neighbor_forward_lane_ids = current_lane.left_neighbor_forward_lane_id
        left_neighbor_forward_lane_lst = []
        for item in left_neighbor_forward_lane_ids:
            left_neighbor_forward_lane_id = item.id
            if driving_only:
                if self.is_driving_lane(left_neighbor_forward_lane_id):
                    left_neighbor_forward_lane_lst.append(left_neighbor_forward_lane_id)
            else:
                left_neighbor_forward_lane_lst.append(left_neighbor_forward_lane_id)
        return left_neighbor_forward_lane_lst

    def get_left_neighbor_reverse_lanes(self, lane_id: str, driving_only=False) -> List[str]:
        current_lane = self.__lanes[lane_id]
        left_neighbor_reverse_lane_ids = current_lane.left_neighbor_reverse_lane_id
        left_neighbor_reverse_lane_lst = []
        for item in left_neighbor_reverse_lane_ids:
            left_neighbor_reverse_lane_id = item.id
            if driving_only:
                if self.is_driving_lane(left_neighbor_reverse_lane_id):
                    left_neighbor_reverse_lane_lst.append(left_neighbor_reverse_lane_id)
            else:
                left_neighbor_reverse_lane_lst.append(left_neighbor_reverse_lane_id)
        return left_neighbor_reverse_lane_lst

    def get_right_neighbor_forward_lanes(self, lane_id: str, driving_only=False) -> List[str]:
        current_lane = self.__lanes[lane_id]
        right_neighbor_forward_lane_ids = current_lane.right_neighbor_forward_lane_id
        right_neighbor_forward_lane_lst = []
        for item in right_neighbor_forward_lane_ids:
            right_neighbor_forward_lane_id = item.id
            if driving_only:
                if self.is_driving_lane(right_neighbor_forward_lane_id):
                    right_neighbor_forward_lane_lst.append(right_neighbor_forward_lane_id)
            else:
                right_neighbor_forward_lane_lst.append(right_neighbor_forward_lane_id)
        return right_neighbor_forward_lane_lst

    def get_right_neighbor_reverse_lanes(self, lane_id: str, driving_only=False) -> List[str]:
        current_lane = self.__lanes[lane_id]
        right_neighbor_reverse_lane_ids = current_lane.right_neighbor_reverse_lane_id
        right_neighbor_reverse_lane_lst = []
        for item in right_neighbor_reverse_lane_ids:
            right_neighbor_reverse_lane_id = item.id
            if driving_only:
                if self.is_driving_lane(right_neighbor_reverse_lane_id):
                    right_neighbor_reverse_lane_lst.append(right_neighbor_reverse_lane_id)
            else:
                right_neighbor_reverse_lane_lst.append(right_neighbor_reverse_lane_id)
        return right_neighbor_reverse_lane_lst

    ######### getter - junction #########
    def get_junction(self, j_id: str) -> Junction:
        """
        Get a specific junction object based on ID

        :param str j_id: ID of the junction interested in

        :returns: junction object
        :rtype: Junction
        """
        return self.__junctions[j_id]

    def get_junction_by_lane_id(self, lane_id):
        """
        Return the junction id where the lane in the junction
        """
        for k, v in self.__lanes_at_junction.items():
            if lane_id in v:
                return k
        return None

    ######### getter - crosswalk #########
    def get_crosswalk(self, cw_id: str) -> Crosswalk:
        """
        Get a specific crosswalk object based on ID

        :param str cw_id: ID of the crosswalk interested in

        :returns: crosswalk object
        :rtype: Crosswalk
        """
        return self.__crosswalk[cw_id]

    def get_crosswalk_polygon(self, cw_id: str) -> Polygon:
        cw = self.__crosswalk[cw_id]
        cw_polygon_points = cw.polygon.point
        polygon_coords = []
        for point in cw_polygon_points:
            polygon_coords.append([point.x, point.y])
        start_point = polygon_coords[0]
        polygon_coords.append(start_point)
        cw_polygon = Polygon(polygon_coords)
        return cw_polygon

    ######### getter - signal #########
    def get_signal(self, s_id: str) -> Signal:
        """
        Get a specific signal object based on ID

        :param str s_id: ID of the signal interested in

        :returns: signal object
        :rtype: Signal
        """
        return self.__signals[s_id]

    ######### getter - stop sign #########
    def get_stop_sign_by_id(self, ss_id: str) -> StopSign:
        """
        Get a specific stop sign object based on ID

        :param str ss_id: ID of the stop sign interested in

        :returns: stop sign object
        :rtype: StopSign
        """
        return self.__stop_signs[ss_id]

    ######### getter - low-level #########
    def get_overlap(self, over_id: str):
        return self.__overlaps[over_id]

    ###### other tools ######
    def get_waypoint(self, lane_id: str, s: float, l: float) -> Waypoint:
        """
        Given a lane_id and a point on the lane, get the actual coordinate and the heading
        at that point.
        """

        def right_rotation(coord, theta):
            """
            theta : degree
            """
            # theta = math.radians(theta)
            x_o = coord[1]
            y_o = coord[0]
            x_r = x_o * math.cos(theta) - y_o * math.sin(theta)
            y_r = x_o * math.sin(theta) + y_o * math.cos(theta)
            return [y_r, x_r]

        lst = self.get_lane_central_curve(lane_id)  # line string
        # logger.debug('s: {}', s)
        ip = lst.interpolate(s)  # a point

        segments = list(map(LineString, zip(lst.coords[:-1], lst.coords[1:])))
        # logger.debug('ip: type {} {}', type(ip), ip)
        segments.sort(key=lambda t: ip.distance(t))
        line = segments[0]
        x1, x2 = line.xy[0]
        y1, y2 = line.xy[1]

        heading = math.atan2(y2 - y1, x2 - x1)

        init_vector = [1, 0]
        right_vector = right_rotation(init_vector, -(heading - math.radians(90.0)))
        x = ip.x + right_vector[0] * l
        y = ip.y + right_vector[1] * l

        wp = Waypoint(
            location=Location(x=x, y=y, z=0, pitch=0, yaw=heading, roll=0),
            lane=Lane(id=lane_id, s=s, l=l)
        )
        return wp