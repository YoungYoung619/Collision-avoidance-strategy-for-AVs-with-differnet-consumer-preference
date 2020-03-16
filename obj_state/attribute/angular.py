"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a class describe one of states in kinematics

Author：Team Li
"""
from msgs.log import logger

class angular(object):
    """a class describe the angular rate of x, y, z in
    real world coordinate.

    Example:
        angular1 = (2., 5., 4.)
        angular2 = (3., 5., 1.)

        ## usage1 ##
        s1 = angular(angular = angular1)
        attr = s1.get()     ##(x, y, z)

        ## usage2 ##
        s2 = geometry_size()
        s2.set(angular = angular2)
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

    def __init__(self, angular=None):
        """init the coordinate attr.
        Args:
            angular: a tuple describes (x, y, z) or (z,) or a float z
        """
        if angular != None:
            try:
                if type(angular) == tuple:
                    assert (len(angular) == 3 or len(angular) == 1)
                    if type(angular) != float:
                        for attr in angular:
                            assert type(attr) == float
                else:
                    assert type(angular) == float
            except AssertionError:
                logger.warning('Pls set angular into (x, y, z), (z,) or z with float, but I '+
                               'get %s'%(str(angular)))
                #print('Warning: pls set angular into (x, y, z), (z,) or z with float!')
            else:
                if type(angular) == tuple:
                    if len(angular) == 3:
                        self.x = angular[0]
                        self.y = angular[1]
                        self.z = angular[2]
                        self.attr = angular
                    elif len(angular) == 1:
                        self.x = 0      ## default setting ##
                        self.y = 0      ## default setting ##
                        self.z = angular[0]
                        self.attr = (self.x, self.y, self.z)
                else:
                    self.x = 0      ## default setting ##
                    self.y = 0      ## default setting ##
                    self.z = angular
                    self.attr = (self.x, self.y, self.z)


    def set(self, angular):
        """set angular rate
        Args:
            angular: a tuple describes (x, y, z) or (z,) or a float z,
                    represents angular rate in each axis.
        """
        try:
            if type(angular) == tuple:
                assert (len(angular) == 3 or len(angular) == 1)
                if type(angular)!=float:
                    for attr in angular:
                        assert type(attr) == float
            else:
                assert type(angular) == float
        except AssertionError:
            logger.warning('Pls set angular into (x, y, z), (z,) or z with float, but I ' +
                           'get %s' % (str(angular)))
            # print('Warning: Pls set angular into (x, y, z), (z,) or z with float!'+
            #       ' But i get %s'%(str(angular)))
        else:
            if type(angular) == tuple:
                if len(angular) == 3:
                    self.x = angular[0]
                    self.y = angular[1]
                    self.z = angular[2]
                    self.attr = angular
                elif len(angular) == 1:
                    self.x = 0.
                    self.y = 0.
                    self.z = angular[0]  ## default setting ##
                    self.attr = (self.x, self.y, self.z)
            else:
                self.x = 0.
                self.y = 0.
                self.z = angular  ## default setting ##
                self.attr = (self.x, self.y, self.z)


    def get(self):
        """get angular rate
        Return:
            if nothing error, a tuple represents (x, y, z).
            else, return None
        """
        return self.attr

## for test ##
if __name__ == "__main__":
    a = (2., 4, 3.)
    b = (1.,)
    c = angular(a)
    print(c.get())
    c.set(b)
    print(c.get())