"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a class describe one of states in kinematics

Author：Team Li
"""
from msgs.log import logger


class linear(object):
    """a class describe the linear speed of x, y, z in
    real world coordinate.

    Example:
        linear1 = (2., 5., 4.)
        linear2 = (3., 5., 1.)

        ## usage1 ##
        s1 = linear(linear = linear1)
        attr = s1.get()     ##(x, y, z)

        ## usage2 ##
        s2 = geometry_size()
        s2.set(linear = linear2)
        attr = s2.get()     ##(x, y, z)

        ## usage3 ##
        x = s2.x
        y = s2.y
        z = s2.z
        attr = s2.attr      ##(x, y, z)
    """

    x = None        ## m/s
    y = None        ## m/s
    z = None        ## m/s
    attr = None  ## (x, y, z)

    def __init__(self, linear=None):
        """init the coordinate attr.
        Args:
            linear: a tuple describes the linear in (x, y, z) or (x, y)
        """
        if linear != None:
            try:
                assert (len(linear) == 3 or len(linear) == 2) \
                       and type(linear) == tuple
                for attr in linear:
                    assert type(attr) == float
            except AssertionError:
                logger.warning('Pls set linear into (x, y, z) or (x, y) with float, but I ' +
                               'get %s' % (str(linear)))
                # print('Warning: pls set linear into (x, y, z) or (x, y) with float!'+
                #       ' But i get %s' % (str(linear)))
            else:
                if len(linear) == 3:
                    self.x = linear[0]
                    self.y = linear[1]
                    self.z = linear[1]
                    self.attr = linear
                else:
                    self.x = linear[0]
                    self.y = linear[1]
                    self.z = 0. ## default setting ##
                    self.attr = self.attr = (self.x, self.y, self.z)


    def set(self, linear):
        """set linear attr
        Args:
            linear: a tuple describes (x, y, z) or (x, y)
        """
        try:
            assert (len(linear) == 3 or len(linear) == 2) \
                   and type(linear) == tuple
            for attr in linear:
                assert type(attr) == float
        except AssertionError:
            logger.warning('Pls set linear into (x, y, z) or (x, y) with float, but I ' +
                           'get %s' % (str(linear)))
        else:
            if len(linear) == 3:
                self.x = linear[0]
                self.y = linear[1]
                self.z = linear[2]
                self.attr = linear
            else:
                self.x = linear[0]
                self.y = linear[1]
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
    a = (2., 4., 3.)
    b = (1., 3.)
    c = linear(a)
    print(c.get())
    c.set(b)
    print(c.get())