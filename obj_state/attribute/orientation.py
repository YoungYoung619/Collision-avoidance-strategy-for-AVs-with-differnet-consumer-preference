"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a class describe one of states in kinematics

Author：Team Li
"""

import math
from msgs.log import logger


class orientation(object):
    """a class describe orientation, only support quaternion format,
    see also https://en.wikipedia.org/wiki/Quaternion

    Quaternion:
        Use four numbers to represent the axis and angle of rotation.
        if the rotation axis is a unit vector (kx, ky, kz), and the
        rotation angle is theta, the quaternion (x, y, z, w) is ca-
        culated as follow:
                x = kx * sin(theta/2)
                y = ky * sin(theta/2)
                z = kz * sin(theta/2)
                w = cos(theta/2)

    Example:
        orientation1 = (2., 5., 4., 5.)  ##just an example, not the real Quaternion val
        orientation2 = (3., 5., 1., 1.)

        ## usage1 ##
        s1 = orientation(orientation = orientation1)
        attr = s1.get()     ##(x, y, z, w)

        ## usage2 ##
        s2 = geometry_size()
        s2.set(orientation = orientation2)
        attr = s2.get()     ##(x, y, z, w)

        ## usage3 ##
        x = s2.x
        y = s2.y
        z = s2.z
        w = s2.w
        attr = s2.attr      ##(x, y, z, w)
    """

    x = None
    y = None
    z = None
    w = None
    attr = None     ##(x, y, z, w)

    def __init__(self, orientation=None):
        """init the orientation attr.
        Args:
            orientation: a tuple describes quaternion (x, y, z, w)
        """
        if orientation != None:
            try:
                assert (len(orientation) == 4 and type(orientation) == tuple)
                for attr in orientation:
                    assert type(attr) == float
            except AssertionError:
                logger.warning('Pls set orientation into (x, y, z, w) with float, but I ' +
                               'get %s' % (str(orientation)))
                # print('Warning: pls set orientation into (x, y, z, w) with float!'+
                #       ' But i get %s' % (str(orientation)))
            else:
                self.x = orientation[0]
                self.y = orientation[1]
                self.z = orientation[2]
                self.w = orientation[3]
                self.attr = orientation


    def set(self, orientation):
        """set orientation attr
        Args:
            orientation: a tuple describes (x, y, z, w)
        """
        try:
            assert (len(orientation) == 4 and type(orientation) == tuple)
            for attr in orientation:
                assert type(attr) == float
        except AssertionError:
            logger.warning('Pls set orientation into (x, y, z, w) with float, but I ' +
                           'get %s' % (str(orientation)))
        else:
            self.x = orientation[0]
            self.y = orientation[1]
            self.z = orientation[2]
            self.w = orientation[3]
            self.attr = orientation

    def get(self):
        """get geometry size
        Return:
            if nothing error, a tuple represents (x, y, z, w).
            else, return None
        """
        return self.attr


if __name__ == "__main__":
    a = (1., 2., 3., 4.)
    d = orientation()
    d.set(a)
    print(d.get())