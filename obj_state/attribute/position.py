"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a class describe one of states in kinematics

Author：Team Li
"""
from msgs.log import logger


class position(object):
    """a class describe the x, y, z in
    real world coordinate.

    Example:
        position1 = (2., 5., 4.)
        position2 = (3., 5., 1.)

        ## usage1 ##
        s1 = position(position = position1)
        attr = s1.get()     ##(x, y, z)

        ## usage2 ##
        s2 = geometry_size()
        s2.set(position = position2)
        attr = s2.get()     ##(x, y, z)

        ## usage3 ##
        x = s2.x
        y = s2.y
        z = s2.z
        attr = s2.attr      ##(x, y, z)
    """

    x = None
    y = None
    z = None
    attr = None  ## (x, y, z)

    def __init__(self, position=None):
        """init the coordinate attr.
        Args:
            position: a tuple describes (x, y, z) or (x, y)
        """
        if position != None:
            try:
                assert (len(position) == 3 or len(position) == 2) \
                       and type(position) == tuple
                for attr in position:
                    assert type(attr) == float
            except AssertionError:
                logger.warning('Pls set position into (x, y, z) or (x, y) with float, but I ' +
                               'get %s' % (str(position)))
                # print('Warning: pls set position into (x, y, z) or (x, y) with float!'+
                #       ' But i get %s' % (str(position)))
            else:
                if len(position) == 3:
                    self.x = position[0]
                    self.y = position[1]
                    self.z = position[1]
                    self.attr = position
                else:
                    self.x = position[0]
                    self.y = position[1]
                    self.z = 0. ## default setting ##
                    self.attr = self.attr = (self.x, self.y, self.z)


    def set(self, position):
        """set position attr
        Args:
            position: a tuple describes (x, y, z) or (x, y)
        """
        try:
            assert (len(position) == 3 or len(position) == 2) \
                   and type(position) == tuple
            for attr in position:
                assert type(attr) == float
        except AssertionError:
            logger.warning('Pls set position into (x, y, z) or (x, y) with float, but I ' +
                           'get %s' % (str(position)))
        else:
            if len(position) == 3:
                self.x = position[0]
                self.y = position[1]
                self.z = position[2]
                self.attr = position
            else:
                self.x = position[0]
                self.y = position[1]
                self.z = 0.  ## default setting ##
                self.attr = (self.x, self.y, self.z)

    def get(self):
        """get geometry size
        Return:
            if nothing error, a tuple represents (x, y, z).
            else, return None
        """
        return self.attr

## for test ##
if __name__ == "__main__":
    a = (2., 3., 3)
    b = (1., 3.)
    c = position(a)
    print(c.get())
    c.set(b)
    print(c.get())