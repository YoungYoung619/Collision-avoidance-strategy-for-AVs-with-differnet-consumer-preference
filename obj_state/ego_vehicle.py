"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
A package wihch provides a class describe the state of ego vehicle

Author：Team Li
"""
from obj_state.road_obj import road_obj
from obj_state.road_obj import obj_type


class ego_vehicle(road_obj):
    """a class describe ego vehicle state,
    inherit from road_obj class.
    """
    throttle = None     ##between [-1,1]
    brake = None
    steer = None        ##between [-1,1]

    def __init__(self, ):
        """init
        """
        road_obj.__init__(self)
        self.set_obj_type(obj_type = obj_type.vehicle)


    def set_throttle(self, throttle):
        """set throttle
        Args:
            throttle: a float
        """
        assert type(throttle) == float
        self.throttle = throttle


    def set_brake(self, brake):
        """set brake
        Args:
            brake: a float
        """
        assert type(brake) == float
        self.brake = float


    def set_steer(self, steer):
        """set steer
        Args:
            steer: a float
        """
        assert type(steer) == float
        self.steer = steer


    def get_throttle(self):
        """get throttle
        Return:
            A float
        """
        return self.throttle


    def get_brake(self):
        """get brake
        Reuturn:
            A float
        """
        return self.brake


    def get_steer(self):
        """get steer
        Return:
            A float
        """
        return self.steer