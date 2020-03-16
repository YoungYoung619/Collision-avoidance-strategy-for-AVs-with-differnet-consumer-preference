"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :


Author：Team Li
"""

import os, sys, glob
try:
    sys.path.append('F:\my_project\driving-desicion-in-carla\dist/carla-0.9.4-py3.7-win-amd64.egg')
    import carla
except:
    raise ImportError('Please check your carla file')

from carla_utils.logging import logger
from carla_utils.world_ops import *
from carla_utils.sensor_ops import *

import msgs.scene_msg as scene_m
from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o
from msgs.log import logger
from situation_assessment import _assess_one_obj_threat_score

from kp2hm_utils import *
import numpy as np

road_range = {'x':(-40, 154), 'y':(204, 207)}


def random_spawn_autopilot_at(world, distance_freq):
    """ random spawn an autopilot at a transform in carla world
    Args:
        world: carla world instance
        distance_freq: a distance control the vehicle number in this road area
    """
    x_min = road_range['x'][0]
    x_max = road_range['x'][1]
    x_dis = x_max - x_min
    freq = x_dis // distance_freq

    vehicles = []
    for i in range(freq-1):
        x = random.randint(x_min + 10 + i*distance_freq, x_min + 10 + (i+1)*distance_freq - 10)
        y = random.sample(road_range['y'], 1)[0]
        if x > 120:
            z = 3
        else:
            z = 1.81
        point = carla.Transform()
        point.location.x = x
        point.location.y = y
        point.location.z = z
        point.rotation.yaw = -0.142975
        vehicle = spawn_egopilot_at(world, point)
        vehicles.append(vehicle)

    return vehicles


def control_vehicles_go_straight_task(vehicles):
    """make vehicles go straight"""
    for vehicle in vehicles:
        vehicle.apply_control(carla.VehicleControl(throttle=random.uniform(0.1, 0.3), steer=0., brake=0.))


def random_spawn_obstacles_in_specific_area(world):
    vehicles = None
    while not vehicles:
        vehicles = random_spawn_autopilot_at(world, 20)

    return vehicles


def nearby_vehicles_info_logger(ego_vehicle, other_traffic_participants, info_type):
    """log the driving situation"""
    assert info_type in ['danger', 'warning']
    pass


def produce_heat_map(ego, others, h_type, hm_size=(224, 224), consider_range=50):
    """produce heat map for each safety degree
    Args:
        ego: ego vehecle in carla
        others: other actor in carla
    """
    assert h_type in ['danger', 'attentive', 'safe']
    ego_location = ego.get_location()
    ego_location = (ego_location.x, ego_location.y, ego_location.z)
    ego_size = ego.bounding_box.extent
    ego_size = (ego_size.x, ego_size.y, ego_size.z)
    ego_velocity = ego.get_velocity()

    ego_velocity = (ego_velocity.x, ego_velocity.y, ego_velocity.z)

    t = ego.get_transform()
    f_v = t.get_forward_vector()

    cos_theta = (f_v.x*0 + f_v.y*1)/math.sqrt(f_v.x**2+ f_v.y**2)
    a = math.acos(cos_theta)
    if f_v.x > 0:
        a = -a

    r_matix = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    # points = []
    # sizes = []
    hms = []
    for vehicle in others:
        location = vehicle.get_location()
        location = (location.x, location.y, location.z)

        distance = math.sqrt((ego_location[0] - location[0]) ** 2 + (ego_location[1] - location[1]) ** 2)
        if distance <= consider_range:
            size = vehicle.bounding_box.extent
            size = (size.x, size.y, size.z)
            velocity = vehicle.get_velocity()
            velocity = (velocity.x, velocity.y, velocity.z)

            ego_v_state = ego_v.ego_vehicle()
            ego_v_state.set_position(position=ego_location)
            ego_v_state.set_linear(linear=ego_velocity)
            ego_v_state.set_size(size=ego_size)

            road_obj_state = road_o.road_obj()
            road_obj_state.set_position(position=location)
            road_obj_state.set_linear(linear=velocity)
            road_obj_state.set_size(size=size)

            safety_degree = _assess_one_obj_threat_score(ego_vehicle=ego_v_state, road_obj=road_obj_state)
            max_index = np.argmax(np.array(safety_degree))
            if max_index == ['danger', 'attentive', 'safe'].index(h_type):
                relative_x = int(location[0] - ego_location[0])*(hm_size[1]//consider_range//2)
                relative_y = int(location[1] - ego_location[1])*(hm_size[0]//consider_range//2)

                point = np.matmul(np.array([relative_x, relative_y]), r_matix)

                point_x = min(hm_size[1]-1, max(-point[0] + hm_size[1]//2, 0))
                point_y = min(hm_size[0]-1, max(-point[1] + hm_size[0]//2, 0))

                size = vehicle.bounding_box.extent
                size = math.sqrt(size.x**2+size.y**2)
                # points.append([point_x, point_y])
                # sizes.append(size/3)
                hm = heat_map(hm_size, points=[[point_x, point_y]], sigma=size*2)
                hm *= safety_degree[max_index]
                hms.append(hm)
    if len(hms) > 0:
        hm = np.sum(np.array(hms), axis=0)
        return hm
    else:
        return np.zeros(hm_size)


if __name__ == '__main__':

    #### carla world init ####
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)  # seconds
    logger.info('Carla connect success...')

    logger.info('Carla world initing...')
    world = client.get_world()

    destroy_all_actors(world)

    init_point = carla.Transform()
    init_point.location.x = -40
    init_point.location.y = random.sample(road_range['y'], 1)[0]
    init_point.location.z = 0.3
    init_point.rotation.yaw = -0.142975
    # ego = spawn_autopilot_at(world, init_point)
    vehicles = random_spawn_autopilot_at(world, 20)

    # a = ego.attributes
    # while True:
    #     s_hm = produce_heat_map(ego, vehicles, h_type='safe')
    #     a_hm = produce_heat_map(ego, vehicles, h_type='attentive')
    #     d_hm = produce_heat_map(ego, vehicles, h_type='danger')
    #     cv2.imshow('safe', s_hm)
    #     cv2.imshow('attentive', a_hm)
    #     cv2.imshow('danger', d_hm)
    #     cv2.waitKey(10)

        # ego_location = ego.get_location()
        # ego_location = (ego_location.x, ego_location.y, ego_location.z)
        # ego_size = ego.bounding_box.extent
        # ego_size = (ego_size.x, ego_size.y, ego_size.z)
        # ego_velocity = ego.get_velocity()
        #
        # t = ego.get_transform()
        # tt = t.get_forward_vector()
        #
        # ego_velocity = (ego_velocity.x, ego_velocity.y, ego_velocity.z)
        #
        # for vehicle in vehicles:
        #     location = vehicle.get_location()
        #     location = (location.x, location.y, location.z)
        #
        #     distance = math.sqrt((ego_location[0]-location[0])**2 + (ego_location[1]-location[1])**2)
        #     if distance <= 100:
        #         size = vehicle.bounding_box.extent
        #         size = (size.x, size.y, size.z)
        #         velocity = vehicle.get_velocity()
        #         velocity = (velocity.x, velocity.y, velocity.z)
        #
        #         ego_v_state = ego_v.ego_vehicle()
        #         ego_v_state.set_position(position=ego_location)
        #         ego_v_state.set_linear(linear=ego_velocity)
        #         ego_v_state.set_size(size=ego_size)
        #
        #         road_obj_state = road_o.road_obj()
        #         road_obj_state.set_position(position=location)
        #         road_obj_state.set_linear(linear=velocity)
        #         road_obj_state.set_size(size=size)
        #
        #         safety_degree = _assess_one_obj_safety(ego_vehicle=ego_v_state, road_obj=road_obj_state)
        #         logger.info("Vehicle_%s safety degree is %s"%(str(vehicle.id), str(safety_degree)))

    pass