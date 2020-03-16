"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
A package wihch provides some class describes the state of road objects when driving.

Author：Team Li
"""
from obj_state.attribute.geometry_size import geometry_size
from obj_state.attribute.orientation import orientation
from obj_state.attribute.position import position
from obj_state.attribute.linear import linear
from obj_state.attribute.angular import angular
from msgs.log import logger

import time
import math
from enum import Enum, unique

# road obj type #
@unique
class obj_type(Enum):
    """usr can set obj type settings
    by their own
    """
    pedestrain = 0
    vehicle = 1
    ## to do ##

## class describe road object ##
class road_obj(object):
    """a class describe the road objs state, all of the states sholud
    be under the ego-vehcile coordinate
    Example:
       ## init some vars ##
        size = (1., 2., 3.)
        position = (2., 3., 4.)
        orientation = (1., 0., 0., 0.)
        linear = (1., 2., 3.)
        angular = 3. ## default in axis z
        t = time.time()
        obj_type = obj_type.vehicle
        obj = road_obj()

        ## set attr ##
        obj.set_size(size)
        obj.set_position(position)
        obj.set_linear(linear)
        obj.set_orientation(orientation)
        obj.set_angular(angular)
        obj.set_time_stamp(t)
        obj.set_obj_type(obj_type)

        ## get attr ##
        s = obj.get_size()

        pt = obj.get_position()
        l = obj.get_linear()
        ps1 = obj.get_orientation(format = 'rpy')
        ps2 = obj.get_orientation(format = 'xyzw')
        a = obj.get_angular()
        t = obj.get_time_stamp()
        type = obj.get_type()
    """
    size = None     ## geometry size
    position = None     ## position
    orientation = None     ## orientation
    linear = None       ##linear speed
    angular = None      ##angular rate
    time_stamp = None   ##record current time
    obj_type = None

    def __init__(self):
        """init some vars
        """
        self.size = geometry_size()
        self.position = position()
        self.orientation = orientation()
        self.linear = linear()
        self.angular = angular()


    def set_size(self, size):
        """ set obj size
        Args:
            size: must be a tuple describes (length, width, height)
        """
        self.size.set(size)


    def set_position(self, position):
        """ set obj position
        Args:
            position: must be a tuple describes (x, y, z) or (x, y)
        """
        self.position.set(position)


    def set_linear(self, linear):
        """ set obj position
        Args:
            linear: must be a tuple describes the speed in (x, y, z) or (x, y), ## m/s
        """
        self.linear.set(linear)


    def set_orientation(self, orientation):
        """set obj orientation
        Args:
            orientation: must be a tuple describes (x, y, z, w)
        """
        self.orientation.set(orientation)


    def set_angular(self, angular):
        """set angular rate
        Args:
            angular: a tuple describes (x, y, z) or (z,) or a float z,
                    represents angular rate in each axis.
        """
        self.angular.set(angular)


    def set_time_stamp(self, time):
        """set time stamp
        Args:
            time: must be a float, seted by time.time(), in sec.
        """
        try:
            assert type(time) == float
        except AssertionError:
            logger.warning('Pls set time through time.time()')
        self.time_stamp = time


    def set_obj_type(self, obj_type):
        """set obj type
        Args:
            obj_type: must be one of Enum obj_type
        """
        self.obj_type = obj_type


    def get_size(self):
        """
        Return:
            a tuple describes size (height, width, depth)
        """
        return self.size.get()


    def get_position(self):
        """
        Return:
            a tuple describes position (x, y, z)
        """
        return self.position.get()


    def get_linear(self):
        """
        Return:
            a tuple describes the speed in (x, y, z)
        """
        return self.linear.get()


    def get_orientation(self, format):
        """get orientation info
        Args:
            format: must be a str in ['xyzw', 'rpy'], 'xyzw' means quaternion,
            'rpy' means  eulerian angle.
        Return:
            a tuple corresponding to format
        """
        try:
            assert format.lower() in ['xyzw', 'rpy']
        except AssertionError:
            logger.error('Pls ensure format is "xyzw" or "rpy"')
            exit(1)

        if format.lower() == 'xyzw':
            return self.orientation.get()
        else:
            ## [-pi, pi] ##
            r = math.atan2(2*(self.orientation.x*self.orientation.w + self.orientation.y*self.orientation.z),
                           1-2*(self.orientation.x**2 + self.orientation.y**2))

            ## [-pi/2, pi/2] ##
            p = math.asin(2*(self.orientation.w*self.orientation.y - self.orientation.x*self.orientation.z))

            ## [-pi, pi] ##
            y = math.atan2(2*(self.orientation.w*self.orientation.z + self.orientation.x*self.orientation.y),
                           1-2*(self.orientation.y**2 + self.orientation.z**2))
            return (r, p, y)


    def get_angular(self):
        """
        Return:
            a tuple represents the angular rate in axis x, y, z
        """
        return self.angular.get()


    def get_type(self):
        """get obj type
        """
        return self.obj_type


    def get_time_stamp(self):
        """get time stamp
        """
        return self.time_stamp