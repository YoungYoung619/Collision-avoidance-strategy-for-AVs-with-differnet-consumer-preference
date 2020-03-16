from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o

from enum import Enum, unique

# describe the weather type #
@unique
class weather(Enum):
    clear = 0
    rain = 1
    fog = 2
    dust = 3

# describe the road state #
@unique
class road_state(Enum):
    normal = 0
    wetness = 1
    snow = 2
    leaves = 3


class scene_msg(object):
    """a class describe secene info, including ego vehicle info,
    other road objs info.
    """
    ego_vehicle = None      ## an instance of ego_vehicle
    road_objs = None        ## a list of road_obj

    def __init__(self):
        """init
        """
        self.road_objs = []


    def set_ego_vehicle(self, ego_vehicle):
        """ set ego vehicle state
        Args:
            ego_vehicle: a class of ego_vehicle, represents
            the state of ego vehicle.
        """
        ## assert ##
        assert type(ego_vehicle) == ego_v.ego_vehicle
        self.ego_vehicle = ego_vehicle


    def set_road_objs(self, road_objs):
        """ set road objs state
        Args:
            road_objs: a list describe each road obj
        """
        ## assert ##
        assert type(road_objs) == list
        for road_obj in road_objs:
            assert type(road_obj) == road_o.road_obj

        self.road_objs = road_objs


    def get_ego_vehicle(self):
        """get a class describe the state of ego vehicle
        Return:
            a class of obj_state.ego_vehicle.ego_vehicle
        """
        return self.ego_vehicle


    def get_road_objs(self):
        """get a list of class, which describe the state
        of other road obj
        Return:
            a class of obj_state.road_obj.road_obj
        """
        return self.road_objs

