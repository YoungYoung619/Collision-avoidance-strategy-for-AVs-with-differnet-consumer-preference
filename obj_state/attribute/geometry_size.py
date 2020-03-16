"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a class describe one of states in kinematics

Author：Team Li
"""
from msgs.log import logger

class geometry_size(object):
    """this object describe the 3d-geometry size in
    real world, encoded by length, width and height

    Example:
        size1 = (2., 5., 4.)
        size2 = (3., 5., 1.)

        ## usage1 ##
        s1 = geometry_size(size = size1)
        attr = s1.get()     ##(height, width, depth)

        ## usage2 ##
        s2 = geometry_size()
        s2.set(size = size2)
        attr = s2.get()     ##(height, width, depth)

        ## usage3 ##
        h = s2.height
        w = s2.width
        d = s2.depth
        attr = s2.attr      ##(height, width, depth)
    """
    height = None
    width = None
    depth = None
    attr = None     ##(height, width, depth)

    def __init__(self, size=None):
        """init the gemometry size.
        Args:
            size: a tuple describes (height, width, depth)
        """
        if size != None:
            try:
                assert (len(size) == 3 and type(size) == tuple)
                for attr in size:
                    assert type(attr) == float
            except AssertionError:
                logger.warning('Pls set size into (length, width, height) with float, but I ' +
                               'get %s' % (str(size)))
                #print('Warning: pls set size into (length, width, height) with float!')
            else:
                self.height = size[0]
                self.width = size[1]
                self.depth = size[2]
                self.attr = size


    def set(self, size):
        """set geometry size.
        Args:
            size: a tuple describes (length, width, height)
        """
        try:
            assert len(size) == 3 and type(size) == tuple
            for attr in size:
                assert type(attr) == float
        except AssertionError:
            logger.warning('Pls set size into (length, width, height) with float, but I ' +
                           'get %s' % (str(size)))
        else:
            self.height = size[0]
            self.width = size[1]
            self.depth = size[2]
            self.attr = size


    def get(self):
        """get geometry size
        Return:
            if nothing error, a tuple represents (h, w, d).
            else, return None
        """
        return self.attr


## for test ##
if __name__ == "__main__":
    a = (2., 5., 4.)
    b = (2., 5., 4.)
    c = geometry_size(size=a)
    d = geometry_size(b)
    print(c.get())
    print(d.get())

    e = geometry_size()
    e.set(a)
    print(e.get())
    e.set(b)
    print(e.get())

    print("ok")