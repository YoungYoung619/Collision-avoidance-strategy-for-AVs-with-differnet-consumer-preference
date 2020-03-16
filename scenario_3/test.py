import math
import numpy as np

COMFORTABLE_DECELERATION = 1.8 ###m/s/s
AVG_DECELERATION = 4 ### m/s/s
MAX_DECELERATION = 13 ### m/s/s

def comfort_level_scores(acc):
    """ assess the comfort level
    Args:
        acc: the acc val for host vehicle
    """
    def norm(x, mu, sigma):
        """normal gaussian function
        """
        pdf = math.exp(-((x - mu) ** 2) / (2 * sigma ** 2))
        return pdf

    def gaussian_1d(x, sigma=5):
        if x >= 0:
            return np.array([0., 0., 1.])
        else:
            x = math.fabs(x)
            # print(x)
            if x >= MAX_DECELERATION:
                return np.array([1, norm(x, AVG_DECELERATION, 6), norm(x, COMFORTABLE_DECELERATION, 1.5)])
            elif x <= COMFORTABLE_DECELERATION:
                return np.array([norm(x, MAX_DECELERATION, 6), norm(x, AVG_DECELERATION, 1.5), 1])
            else:
                return np.array([norm(x, MAX_DECELERATION, 6), norm(x, AVG_DECELERATION, 1.5), norm(x, COMFORTABLE_DECELERATION, 1.5)])

    return gaussian_1d(acc)

a = comfort_level_scores(-10)
pass