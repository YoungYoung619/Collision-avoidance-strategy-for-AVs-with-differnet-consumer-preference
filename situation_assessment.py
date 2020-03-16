"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a tool to assess the degree of safety when driving,

Author：Team Li
"""
from enum import Enum, unique
import numpy as np
import math

import msgs.scene_msg as scene_m
from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o
from msgs.log import logger

# safety degree #
@unique
class safety_degree(Enum):
    """describ the safety degree
    """
    safe = 0
    attentive = 1
    dangerous = 2


@unique
class relation_state(Enum):
    """describ the relation between ego vehicle and other obj
    Params:
        none_overlap:
        overlap_in_x:
        overlap_in_y:
        all_overlap:
    """
    none_overlap = 0
    overlap_in_x = 1
    overlap_in_y = 2
    all_overlap = 3


## use to calculate theshold of safety degree ##
weather_effect_ratio = {scene_m.weather.clear: 1.,
                        scene_m.weather.fog: 1.4,
                        scene_m.weather.rain: 1.2,
                        scene_m.weather.dust: 1.3}
road_state_ratio = {scene_m.road_state.normal: 1.,
                    scene_m.road_state.wetness: 0.83,
                    scene_m.road_state.snow: 0.67,
                    scene_m.road_state.leaves: 0.91,}

COMFORTABLE_DECELERATION = 1.8 ###m/s/s
AVG_DECELERATION = 4 ### m/s/s
MAX_DECELERATION = 13 ### m/s/s

##ploy used to calculate the std and max of gaussian##
# x = np.arange(0, 70, 10)
# std_y = np.array([0.5, 0.9, 1.3, 1.7, 2.1, 2.5, 2.9])
# max_y = np.array([7., 8., 10., 15., 20., 25., 30.])
# z_std = np.polyfit(x, std_y, 4)
# p_std = np.poly1d(z_std)
# z_max = np.polyfit(x, max_y, 3)
# p_max = np.poly1d(z_max)

def gaussian_1d(x, mean, for_what, std, max):
    """produce a val respresents the socre according the gaussian distribution.
    Args:
        x: a input val.
        mean: the mean of gaussian distribution.
        std: the std of gaussian distribution.
        for_what: should be one of safety_degree
    """

    def norm(x, mu, sigma):
        """normal gaussian function
        """
        #print(sigma)
        pdf = math.exp(-((x - mu) ** 2) / (2 * sigma ** 2)) / (sigma * math.sqrt(2 * np.pi))
        return max*pdf

    assert for_what in list(safety_degree.__members__.values())

    if for_what is safety_degree.dangerous:
        if x < mean:
            score = norm(x=mean, mu=mean, sigma=std)
        else:
            score = norm(x=x, mu=mean, sigma=std)
    elif for_what is safety_degree.attentive:
        score = norm(x=x, mu=mean, sigma=std)
    elif for_what is safety_degree.safe:
        if x > mean:
            score = norm(x=mean, mu=mean, sigma=std)
        else:
            score = norm(x=x, mu=mean, sigma=std)
    else:
        raise ValueError("Error")

    return score


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

def _score_2_threat_degree(score):
    if np.argmax(score) == 0:
        return safety_degree.dangerous
    elif np.argmax(score) == 1:
        return safety_degree.attentive
    else:
        return safety_degree.safe


def _assess_one_obj_threat_score(ego_vehicle, road_obj, weather_type=scene_m.weather.clear,
                           road_state=scene_m.road_state.normal):
    """assess the road object safety degree for ego vehicle
    Args:
        ego_vehicle: a class in obj_state.ego_vehicel
        road_obj: a class in obj_state.road_obj
        weather_type: when in unclear weather, it is more likely to be dangerous
        road_state: when road is wetness, snow or leaves, it is more likely to be dangerous
    Return:
        [Dangerous score, attentive score, safe score]
    """

    def judge_relation_state(ego_pos, ego_size, other_pos, other_size):
        """jugde the relation between ego vehicle and other obj
        Args:
            ego_pos: a tuple describe ego position, (x, y, z)
            ego_size: a tuple describe ego geometry size, (h, w, d)
            other_pos: a tuple describe other vehicle position, (x, y, z)
            other_size: a tuple describe other vehicle geometry size, (h, w, d)
        Return:
            a relation_state
        """
        ego_vehicle_radius = math.sqrt((ego_size[1]/2)**2 + (ego_size[2]/2)**2)
        other_obj_radius = math.sqrt((other_size[1]/2)**2 + (other_size[2]/2)**2)
        length = ego_vehicle_radius + other_obj_radius

        m_distance_x = math.fabs(ego_pos[0] - other_pos[0])
        m_distance_y = math.fabs(ego_pos[1] - other_pos[1])

        if m_distance_x <= length or m_distance_y <= length:
            if m_distance_x <= length and m_distance_y <= length:
                return relation_state.all_overlap
            elif m_distance_x <= length:
                return relation_state.overlap_in_x
            else:
                return relation_state.overlap_in_y
        else:
            return relation_state.none_overlap


    def time_to_collision_2d(ego_pos, ego_linear, ego_size, other_pos, other_linear, other_size, dim='xy'):
        """calculate TTC in x,y dimention, TTC is calculated as: TTC = (x[i-1] - x[i] - L) / (v[i] - v[i-1]),
            where i indicates the vehicle position which has the bigger velocity. When TTC > 0, it indicates
            that the remaining time in which two vehicles would collide. When TTC < 0, in indicates that it's
            impossible for two cars to collide.
        Args:
            ego_pos: a tuple describe ego position, (x, y, z)
            ego_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            ego_size: a tuple describe ego geometry size, (h, w, d)
            other_pos: a tuple describe other vehicle position, (x, y, z)
            other_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            other_size: a tuple describe other vehicle geometry size, (h, w, d)
            dim: a string indicates calculate TTC in which dimention, must be 'xy', 'y' or 'x'.
        Return:
            ttc_in_x: the remaining time in which two vehicles would collide in x dimention.
            ttc_in_y: the remaining time in which two vehicles would collide in y dimention.
        """
        ## assert ##
        assert dim in ['xy', 'x', 'y']

        ## length ##
        ego_vehicle_radius = math.sqrt((ego_size[1] / 2) ** 2 + (ego_size[2] / 2) ** 2)
        other_obj_radius = math.sqrt((other_size[1] / 2) ** 2 + (other_size[2] / 2) ** 2)
        length = ego_vehicle_radius + other_obj_radius

        ## 1e-5 is to avoid zero-divide error ##
        if ego_linear[0] >= other_linear[0]:
            ttc_in_x = (other_pos[0] - ego_pos[0] - length) / (ego_linear[0] - other_linear[0] + 1e-5)
        else:
            ttc_in_x = (ego_pos[0] - other_pos[0] - length) / (other_linear[0] - ego_linear[0] + 1e-5)

        if ego_linear[1] >= other_linear[1]:
            ttc_in_y = (other_pos[1] - ego_pos[1] - length) / (ego_linear[1] - other_linear[1] + 1e-5)
        else:
            ttc_in_y = (ego_pos[1] - other_pos[1] - length) / (other_linear[1] - ego_linear[1] + 1e-5)

        if dim == 'xy':
            return ttc_in_x, ttc_in_y
        elif dim == 'x':
            return ttc_in_x
        elif dim == 'y':
            return ttc_in_y
        else:
            raise ValueError("Imposible get here!!! Something must wrong!!!")


    def much_bigger(val, bigger_than):
        """ normally, the val should be a abs of subtraction value
        Return:
            if val >= bigger_than: True
            else: False
        """
        if val >= bigger_than:
            return True
        else:
            return False


    def time_to_escape_2d(ego_pos, ego_linear, ego_size, other_pos, other_linear, other_size, dim='xy'):
        """When the relation between ego vehicle and other obj is overlap, we need this indicator to
            indicate the remaining time in which the ego(or other obj) can escape the overlap area.
        Args:
            ego_pos: a tuple describe ego position, (x, y, z)
            ego_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            ego_size: a tuple describe ego geometry size, (h, w, d)
            other_pos: a tuple describe other vehicle position, (x, y, z)
            other_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            other_size: a tuple describe other vehicle geometry size, (h, w, d)
            dim: a string indicates calculate TTE in which dimention, must be 'xy', 'y' or 'x'.
        Return:
            TTE in specific dimention
        """
        ## length ##
        ego_vehicle_radius = math.sqrt((ego_size[1] / 2) ** 2 + (ego_size[2] / 2) ** 2)
        other_obj_radius = math.sqrt((other_size[1] / 2) ** 2 + (other_size[2] / 2) ** 2)
        length = ego_vehicle_radius + other_obj_radius

        ## 1e-8 is to avoid zero-divide error ##
        if ego_linear[0] >= other_linear[0]:
            tte_in_x = (other_pos[0] - ego_pos[0] + length) / (ego_linear[0] - other_linear[0] + 1e-8)
        else:
            tte_in_x = (ego_pos[0] - other_pos[0] + length) / (other_linear[0] - ego_linear[0] + 1e-8)

        if ego_linear[1] >= other_linear[1]:
            tte_in_y = (other_pos[1] - ego_pos[1] + length) / (ego_linear[1] - other_linear[1] + 1e-8)
        else:
            tte_in_y = (ego_pos[1] - other_pos[1] + length) / (other_linear[1] - ego_linear[1] + 1e-8)

        if dim == 'xy':
            return tte_in_x, tte_in_y
        elif dim == 'x':
            return tte_in_x
        elif dim == 'y':
            return tte_in_y
        else:
            raise ValueError("Imposible get here!!! Something must wrong!!!")

    def softmax(x):
        return np.exp(x) / np.expand_dims(np.sum(np.exp(x), axis=-1),axis=-1)

    def cos_vec(vec1, vec2):
        """caculate the cos val between two vec
        Args:
            vec1: ndarray with a shape [n_dim]
            vec2: ndarray with a shape [n_dim]
        """
        cos = np.sum(vec1*vec2) / (np.sqrt(np.sum(np.square(vec1)))*np.sqrt(np.sum(np.square(vec2))) +1e-6)
        return cos


    ## assert ##
    assert type(ego_vehicle) == ego_v.ego_vehicle
    assert type(road_obj) == road_o.road_obj
    assert weather_type in list(scene_m.weather.__members__.values())
    assert road_state in list(scene_m.road_state.__members__.values())

    ## init safety degree score ##
    safety_score = np.zeros(shape=[len(list(safety_degree.__members__.values()))]) ##

    ego_pos = ego_vehicle.get_position()
    ego_linear = ego_vehicle.get_linear()
    ego_size = ego_vehicle.get_size()
    other_pos = road_obj.get_position()
    other_size = road_obj.get_size()
    other_linear = road_obj.get_linear()

    r_state = judge_relation_state(ego_pos, ego_size, other_pos, other_size)
    # print('r_state:', r_state)

    if r_state is relation_state.none_overlap:
        ttc_in_x, ttc_in_y = time_to_collision_2d(ego_pos, ego_linear, ego_size,
                                                  other_pos, other_linear, other_size)

        if ttc_in_x < 0. or ttc_in_y < 0.:
            ## safe ##
            return np.array([0., 0., 1.]), -1.
        else:
            vel_m_s = math.sqrt(ego_linear[0] ** 2 + ego_linear[1] ** 2)  ## m/s
            vel_m_s_x = math.fabs(ego_linear[0])
            vel_m_s_y = math.fabs(ego_linear[1])

            c_t_thresh = (vel_m_s/COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]
            c_t_thresh_x = (vel_m_s_x / COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                weather_type]
            c_t_thresh_y = (vel_m_s_y / COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                weather_type]

            ## maybe unsafe ##
            if much_bigger(math.fabs(ttc_in_y - ttc_in_x), c_t_thresh):
                ## safe ##
                return np.array([0., 0., 1.]), -1.
            elif much_bigger(ttc_in_x, c_t_thresh_x) and much_bigger(ttc_in_y, c_t_thresh_y):
                ## safe ##
                return np.array([0., 0., 1.]), -1.
            else:
                ## safe, attentive or dagerous ##
                ##  we use the speedofometer to calculate the threshold of different safety degree
                # vel_km_h = vel_m_s / .27777 ## km/h
                # thresh_w = np.array(safe_distance_config[weather_type])*vel_km_h/vel_m_s  ##add safe_distance factor and weatehr factor
                # thresh_ratio_r = road_state_ratio[road_state]       ##add road
                # thresh = thresh_w*thresh_ratio_r  ##add

                thresh = [(vel_m_s/(MAX_DECELERATION*road_state_ratio[road_state])*weather_effect_ratio[weather_type]),
                          (vel_m_s/(AVG_DECELERATION*road_state_ratio[road_state])*weather_effect_ratio[weather_type]),
                          (vel_m_s/(COMFORTABLE_DECELERATION*road_state_ratio[road_state])*weather_effect_ratio[weather_type])]

                dangerous_score = gaussian_1d(x=max(ttc_in_y, ttc_in_x),
                                              mean=thresh[0], for_what=safety_degree.dangerous,
                                              std=(vel_m_s*3.6)/20, max=(vel_m_s*3.6)/2)
                attentive_score = gaussian_1d(x=max(ttc_in_y, ttc_in_x),
                                              mean=thresh[1], for_what=safety_degree.attentive,
                                              std=(vel_m_s*3.6)/20, max=(vel_m_s*3.6)/2)
                safe_score = gaussian_1d(x=max(ttc_in_y, ttc_in_x),
                                         mean=thresh[2], for_what=safety_degree.safe,
                                         std=(vel_m_s*3.6)/20, max=(vel_m_s*3.6)/2)

                return softmax(np.array([dangerous_score, attentive_score, safe_score])), -1.

    elif r_state is relation_state.overlap_in_x:
        tte_in_x = time_to_escape_2d(ego_pos, ego_linear, ego_size,
                                     other_pos, other_linear, other_size, dim='x')
        ttc_in_y = time_to_collision_2d(ego_pos, ego_linear, ego_size,
                                     other_pos, other_linear, other_size, dim='y')

        try:
            assert tte_in_x >= 0.
        except AssertionError:
            logger.warning('Get a negative tte val(%f) in x, something may be wrong!'%(tte_in_x))

        if ttc_in_y < 0.:
            ## safe ##
            return np.array([0., 0., 1.]), -1.
        else:
            vel_m_s = math.sqrt(ego_linear[0] ** 2 + ego_linear[1] ** 2)  ## m/s
            vel_m_s_x = math.fabs(ego_linear[0] - other_linear[1])
            vel_m_s_y = math.fabs(ego_linear[1] - other_linear[1])

            # if much_bigger(ttc_in_y - tte_in_x,
            #                vel_m_s/(COMFORTABLE_DECELERATION*road_state_ratio[road_state]*weather_effect_ratio[weather_type])):
            #     ## safe ##
            #     return np.array([0., 0., 1.])
            if tte_in_x >= ttc_in_y:
                if much_bigger(ttc_in_y,
                               (vel_m_s_y / ((COMFORTABLE_DECELERATION * road_state_ratio[road_state])) \
                                            * weather_effect_ratio[weather_type])):
                    ## safe ##
                    return np.array([0., 0., 1.]), ttc_in_y
                else:
                    ## safe, attentive, dangerous ##
                    thresh = [
                        (vel_m_s_y / (MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                        (vel_m_s_y / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                        (vel_m_s_y / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                            weather_type])]

                    dangerous_score = gaussian_1d(x=ttc_in_y,
                                                  mean=thresh[0], for_what=safety_degree.dangerous,
                                                  std=(vel_m_s_y * 3.6) / 20, max=(vel_m_s_y * 3.6) / 2)
                    attentive_score = gaussian_1d(x=ttc_in_y,
                                                  mean=thresh[1], for_what=safety_degree.attentive,
                                                  std=(vel_m_s_y * 3.6) / 20, max=(vel_m_s_y * 3.6)  / 2)
                    safe_score = gaussian_1d(x=ttc_in_y,
                                             mean=thresh[2], for_what=safety_degree.safe,
                                             std=(vel_m_s_y * 3.6) / 20, max=(vel_m_s_y * 3.6)  / 2)

                    return softmax(np.array([dangerous_score, attentive_score, safe_score])), ttc_in_y
            else:
                if much_bigger(ttc_in_y,
                               (vel_m_s_y / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) \
                                            * weather_effect_ratio[weather_type])):
                    ## safe ##
                    return np.array([0., 0., 1.]), ttc_in_y
                else:
                    thresh = [(vel_m_s / (
                                MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                              (vel_m_s / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                                  weather_type]),
                              (vel_m_s / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                                  weather_type])]

                    ## safe attentive dangerous ##
                    dangerous_score = gaussian_1d(x=math.fabs(ttc_in_y - tte_in_x),
                                                  mean=thresh[0], for_what=safety_degree.dangerous,
                                                  std=(vel_m_s * 3.6) / 20, max=(vel_m_s * 3.6) / 2)
                    attentive_score = gaussian_1d(x=math.fabs(ttc_in_y - tte_in_x),
                                                  mean=thresh[1], for_what=safety_degree.attentive,
                                                  std=(vel_m_s * 3.6) / 20, max=(vel_m_s * 3.6) / 2)
                    safe_score = gaussian_1d(x=math.fabs(ttc_in_y - tte_in_x),
                                             mean=thresh[2], for_what=safety_degree.safe,
                                             std=(vel_m_s * 3.6) / 20, max=(vel_m_s * 3.6) / 2)
                    return softmax(np.array([dangerous_score, attentive_score, safe_score])), ttc_in_y


    elif r_state is relation_state.overlap_in_y:
        tte_in_y = time_to_escape_2d(ego_pos, ego_linear, ego_size,
                                     other_pos, other_linear, other_size, dim='y')
        ttc_in_x = time_to_collision_2d(ego_pos, ego_linear, ego_size,
                                        other_pos, other_linear, other_size, dim='x')

        try:
            assert tte_in_y >= 0.
        except AssertionError:
            logger.warning('Get a negative tte val(%f) in y, something may be wrong!' % (tte_in_y))

        if ttc_in_x < 0.:
            ## safe ##
            return np.array([0., 0., 1.]), ttc_in_x
        else:
            vel_m_s = math.sqrt(ego_linear[0] ** 2 + ego_linear[1] ** 2)  ## m/s
            vel_m_s_x = math.fabs(ego_linear[0] - other_linear[0])
            vel_m_s_y = math.fabs(ego_linear[1] - other_linear[0])

            # if much_bigger(ttc_in_y - tte_in_x,
            #                vel_m_s/(COMFORTABLE_DECELERATION*road_state_ratio[road_state]*weather_effect_ratio[weather_type])):
            #     ## safe ##
            #     return np.array([0., 0., 1.])
            if tte_in_y >= ttc_in_x:
                if much_bigger(ttc_in_x,
                               (vel_m_s_x / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) \
                                            * weather_effect_ratio[weather_type])):
                    ## safe ##
                    return np.array([0., 0., 1.]), ttc_in_x
                else:
                    ## safe, attentive, dangerous ##
                    thresh = [
                        (vel_m_s_x / (MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                            weather_type]),
                        (vel_m_s_x / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                            weather_type]),
                        (vel_m_s_x / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                            weather_type])]

                    dangerous_score = gaussian_1d(x=ttc_in_x,
                                                  mean=thresh[0], for_what=safety_degree.dangerous,
                                                  std=(vel_m_s_x * 3.6) / 20, max=(vel_m_s_x * 3.6) / 2)
                    attentive_score = gaussian_1d(x=ttc_in_x,
                                                  mean=thresh[1], for_what=safety_degree.attentive,
                                                  std=(vel_m_s_x * 3.6) / 20, max=(vel_m_s_x * 3.6) / 2)
                    safe_score = gaussian_1d(x=ttc_in_x,
                                             mean=thresh[2], for_what=safety_degree.safe,
                                             std=(vel_m_s_x * 3.6) / 20, max=(vel_m_s_x * 3.6) / 2)

                    return softmax(np.array([dangerous_score, attentive_score, safe_score])), ttc_in_x
            else:
                if much_bigger(ttc_in_x,
                               (vel_m_s_x / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) \
                                            * weather_effect_ratio[weather_type])):
                    ## safe ##
                    return np.array([0., 0., 1.]),ttc_in_x
                else:
                    thresh = [(vel_m_s / (
                            MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                              (vel_m_s / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                                  weather_type]),
                              (vel_m_s / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                                  weather_type])]

                    ## safe attentive dangerous ##
                    dangerous_score = gaussian_1d(x=math.fabs(ttc_in_x - tte_in_y),
                                                  mean=thresh[0], for_what=safety_degree.dangerous,
                                                  std=(vel_m_s * 3.6) / 20, max=(vel_m_s * 3.6) / 2)
                    attentive_score = gaussian_1d(x=math.fabs(ttc_in_x - tte_in_y),
                                                  mean=thresh[1], for_what=safety_degree.attentive,
                                                  std=(vel_m_s * 3.6) / 20, max=(vel_m_s * 3.6) / 2)
                    safe_score = gaussian_1d(x=math.fabs(ttc_in_x - tte_in_y),
                                             mean=thresh[2], for_what=safety_degree.safe,
                                             std=(vel_m_s * 3.6) / 20, max=(vel_m_s * 3.6) / 2)
                    return softmax(np.array([dangerous_score, attentive_score, safe_score])), ttc_in_x
    elif r_state is relation_state.all_overlap:
        ## length ##
        ego_vehicle_radius = math.sqrt((ego_size[1] / 2) ** 2 + (ego_size[2] / 2) ** 2)
        other_obj_radius = math.sqrt((other_size[1] / 2) ** 2 + (other_size[2] / 2) ** 2)
        length = ego_vehicle_radius + other_obj_radius

        ## pos_vec ##
        ego_pos_vec = np.array([ego_pos[0], ego_pos[1]])
        other_pos_vec = np.array([other_pos[0], other_pos[1]])
        pos_vec = ego_pos_vec - other_pos_vec
        pos_scalar = np.sqrt(np.sum(np.square(pos_vec)))

        ## vel_vec ##
        ego_vel_vec = np.array([ego_linear[0], ego_linear[1]])
        other_vel_vec = np.array([other_linear[0], other_linear[1]])
        vel_vex =  other_vel_vec - ego_vel_vec
        vel_scalar = np.sqrt(np.sum(np.square(vel_vex)))
        ego_vel_scalar = np.sqrt(np.sum(np.square(ego_vel_vec))) ## m/s

        cos = cos_vec(pos_vec, vel_vex)

        if cos <= 0.:
            ##means the distance between ego and other obj would increase.
            return np.array([0., 0., 1.]), -1
        else:
            ##means the distance between ego and other obj would decrease.
            vel_rel = vel_scalar * cos
            ttc = (pos_scalar - length) / vel_rel

            ## safe, attentive, dangerous ##
            thresh = [
                (ego_vel_scalar / (MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                    weather_type]),
                (ego_vel_scalar / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                    weather_type]),
                (ego_vel_scalar / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                    weather_type])]

            dangerous_score = gaussian_1d(x=ttc,
                                          mean=thresh[0], for_what=safety_degree.dangerous,
                                          std=(ego_vel_scalar * 3.6) / 20, max=(ego_vel_scalar * 3.6) / 2)
            attentive_score = gaussian_1d(x=ttc,
                                          mean=thresh[1], for_what=safety_degree.attentive,
                                          std=(ego_vel_scalar * 3.6) / 20, max=(ego_vel_scalar * 3.6) / 2)
            safe_score = gaussian_1d(x=ttc,
                                     mean=thresh[2], for_what=safety_degree.safe,
                                     std=(ego_vel_scalar * 3.6) / 20, max=(ego_vel_scalar * 3.6) / 2)

            return softmax(np.array([dangerous_score, attentive_score, safe_score])), ttc

    else:
        raise ValueError("Imposible get here!!! Something must wrong!!!")


if __name__ == '__main__':
    ego_v_state = ego_v.ego_vehicle()
    ego_v_state.set_position(position=(0., 0., 0.))
    ego_v_state.set_linear(linear=(15., 1., 0.))
    ego_v_state.set_size(size=(1.3, 1.5, 4.))

    road_obj_state = road_o.road_obj()
    road_obj_state.set_position(position=(50., 0., 0.))
    road_obj_state.set_linear(linear=(1., 0., 0.))
    road_obj_state.set_size(size=(1.3, 1.5, 4.))

    safety_degree, ttc = _assess_one_obj_safety(ego_vehicle=ego_v_state, road_obj=road_obj_state)
    pass