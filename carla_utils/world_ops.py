"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
some world query and operation tools

Author：Team Li
"""
import random, time
from carla_utils.logging import logger
import sys, math
# from scenario_1 import config
import config
try:
    sys.path.append(config.carla_egg_file)
    import carla
    from carla import ColorConverter as cc
except:
    raise ImportError('Please check your carla file')

"""
#####  User Recommended Functions  ####
--------------respawn_static_actors
"""

cornet_init_points = [12, 42, 63, 64, 70, 71, 72, 73, 86, 87, 94, 95, 105, 106, 107, 108, 109, 119, 120, 121, 122]

def try_spawn_at(world, vehicle_bp, transform, autopilot):
    """ spawn a vehicle in world
    """
    actor = world.try_spawn_actor(vehicle_bp, transform)
    # aa = list(world.get_actors())
    # bb = world.get_spectator()
    if not actor:
        raise ValueError('could not spawn vehicle...')
    else:
        actor.set_autopilot(autopilot)
        actor.apply_control(carla.VehicleControl(brake=1.))
        return True

def actor_static(actor):
    """judge whether an actor is in static state
    Args:
        actor: an actor of vehicle
    Return:
        a bool val respresents whether the actor is in static state
    """
    vel = actor.get_velocity()
    if abs(vel.x) <= 0.1 and abs(vel.y) <= 0.1 and abs(vel.z) <= 0.1:
        return True
    else:
        return False


def static_rate(actors):
    """get the static rate in carla world
    Args:
        actors: all the vehicle actor list
    Return:
        a rate respresents how many vehicle in static state
    """
    static_n = 0
    for vehicle in actors:
        if actor_static(vehicle):
            static_n += 1
    rate = static_n / len(actors)
    return rate


def carla_actors_static(actors, bigger_than):
    """judge whether all the actors are in static state
    Return:
        a bool val.
    """
    if static_rate(actors) > bigger_than:
        return True
    else:
        return False


def respawn_static_actors(world, actors):
    """re-spawn the static actors in the carla world
    Args:
        world: client.get_world()
        actors:world.get_actors()
    Example:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0) # seconds

        actor_list = world.get_actors()
        vehicles = list(actor_list.filter('vehicle*'))
        if carla_actors_static(vehicles):
            respawn_static_actors(vehicles)
    """
    for vehicle in actors:
        if actor_static(vehicle):
            spawn_points = list(world.get_map().get_spawn_points())
            index = random.randint(0, (len(spawn_points))-1)
            # index = 45
            vehicle.set_transform(spawn_points[index])
            logger.info('Respawn '+str(vehicle)+' in '+str(spawn_points[index]))

def respawn_all_actors(world, actors):
    pass

def is_spawn_point_safe(actors_in_world, spawn_point):
    for actor in actors_in_world:
        location = actor.get_location()
        distance = math.sqrt((location.x - spawn_point.location.x)**2 + (location.y - spawn_point.location.y)**2)
        if distance >= 50:
            return True
    return False


def respawn_actors(world, actors):
    """re-spawn all the actors in the carla world
    Args:
        world: client.get_world()
        actors:world.get_actors()
    Example:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0) # seconds

        actor_list = world.get_actors()
        vehicles = list(actor_list.filter('vehicle*'))
        if carla_actors_static(vehicles):
            respawn_static_actors(vehicles)
    """
    for vehicle in actors:
        v = vehicle.get_velocity()
        a_v = vehicle.get_angular_velocity()

        v.x = 0.
        v.y = 0.
        v.z = 0.

        a_v.x = 0.
        a_v.y = 0.
        a_v.z = 0.

        vehicle.set_velocity(v)
        vehicle.set_angular_velocity(a_v)
        vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=0))
        spawn_points = list(world.get_map().get_spawn_points())
        index = random.randint(0, (len(spawn_points))-1)
        # index = 45
        # spawn_points[index].location.z = spawn_points[index].location.z
        spawn_points[index].location.z = 0.001
        vehicle.set_transform(spawn_points[index])
        #logger.info('Respawn '+str(vehicle)+' in '+str(spawn_points[index]))


def respawn_actor_at(world, actor, transform):
    """re-spawn all the actors in the carla world
    Args:
        world: client.get_world()
        actors:world.get_actors()
    Example:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0) # seconds

        actor_list = world.get_actors()
        vehicles = list(actor_list.filter('vehicle*'))
        if carla_actors_static(vehicles):
            respawn_static_actors(vehicles)
    """

    v = actor.get_velocity()
    a_v = actor.get_angular_velocity()

    v.x = 0.
    v.y = 0.
    v.z = 0.

    a_v.x = 0.
    a_v.y = 0.
    a_v.z = 0.

    actor.set_velocity(v)
    actor.set_angular_velocity(a_v)
    actor.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=0))

    transform.location.z = 0.001
    actor.set_transform(transform)
    #logger.info('Respawn '+str(vehicle)+' in '+str(spawn_points[index]))


def try_spawn_random_vehicle_at(world, transform, autopilot=True):
    blueprints = world.get_blueprint_library().filter('vehicle.*')
    if not autopilot:
        blueprints = world.get_blueprint_library().filter('vehicle.nissan.micra')
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprint = random.choice(blueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)

    if autopilot:
        blueprint.set_attribute('role_name', 'autopilot')
    else:
        blueprint.set_attribute('role_name', 'egopilot')
    vehicle = world.try_spawn_actor(blueprint, transform)
    if (vehicle is not None) and (autopilot):
        vehicle.set_autopilot(True)
        logger.info('spawned a autopilot %r at %s' % (vehicle.type_id, transform.location))
        return True
    elif (vehicle is not None) and (not autopilot):
        vehicle.set_autopilot(False)
        logger.info('spawned a egopilot %r at %s' % (vehicle.type_id, transform.location))
        return True
    return False


def try_spawn_random_pedestrain_at(world, transform):
    blueprints = world.get_blueprint_library().filter('walker*')
    blueprint = random.choice(blueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)

    blueprint.set_attribute('role_name', 'pedestrain')

    pedestrain = world.try_spawn_actor(blueprint, transform)
    if (pedestrain is not None):
        logger.info('spawned a pedestrain %r at %s' % (pedestrain.type_id, transform.location))
        return True
    return False


def spawn_vehicles(world, n_autopilots, n_egopilots, n_pedestrains=0):
    """spawn some vehicles in carla world
    Args:
        n_autopilots: the number of autopilot vehicles spawned in carla world
        n_pedestrains: the number of pedestrains spawned in carla world
        n_egopilots: the number of egopilot vehicles spawned in calla world
    """
    try:
        assert n_pedestrains==0
    except:
        raise NotImplementedError('not support pedestrain now')

    if n_autopilots > 0:
        count = n_autopilots
        spawn_points = list(world.get_map().get_spawn_points())
        random.shuffle(spawn_points)
        for spawn_point in spawn_points:
            if try_spawn_random_vehicle_at(world, spawn_point):
                count -= 1
            if count <= 0:
                break

        while count > 0:
            time.sleep(500)
            if try_spawn_random_vehicle_at(world, random.choice(spawn_points)):
                count -= 1

    if n_pedestrains > 0:
        count = n_pedestrains
        spawn_points = list(world.get_map().get_spawn_points())
        random.shuffle(spawn_points)
        for spawn_point in spawn_points:
            if try_spawn_random_pedestrain_at(world, spawn_point):
                count -= 1
            if count <= 0:
                break

        while count > 0:
            time.sleep(500)
            if try_spawn_random_pedestrain_at(world, random.choice(spawn_points)):
                count -= 1

    if n_egopilots > 0:
        count = n_egopilots
        spawn_points = list(world.get_map().get_spawn_points())
        random.shuffle(spawn_points)
        for spawn_point in spawn_points:
            if try_spawn_random_vehicle_at(world, spawn_point, autopilot=False):
                count -= 1
            if count <= 0:
                break

        while count > 0:
            time.sleep(500)
            if try_spawn_random_vehicle_at(world, random.choice(spawn_points), autopilot=False):
                count -= 1


def spawn_egopilot_at(world, transform):
    blueprints = world.get_blueprint_library().filter('vehicle.nissan.micra')
    blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprint = random.choice(blueprints)
    blueprint.set_attribute('role_name', 'egopilot')
    vehicle = world.try_spawn_actor(blueprint, transform)
    vehicle.set_autopilot(False)
    return vehicle


def spawn_autopilot_at(world, transform):
    blueprints = world.get_blueprint_library().filter('vehicle*')
    blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprint = random.choice(blueprints)
    blueprint.set_attribute('role_name', 'autopilot')
    vehicle = world.try_spawn_actor(blueprint, transform)
    if not vehicle:
        return None
    vehicle.set_autopilot(False)
    return vehicle


def get_all_autopilots(world):
    """get all the autopilot vehicles in carla world
    Return:
        a list represents all the autopilot actors
    """
    autopilots = []

    actor_list = world.get_actors()
    vehicles = list(actor_list.filter('vehicle*'))
    for vehicle in vehicles:
        if vehicle.attributes['role_name'] == 'autopilot':
            autopilots.append(vehicle)
    return autopilots


def get_all_egopilots(world):
    """get all the egopilots vehicles in carla world
    Return:
        a list represents all the egopilots actors
    """
    egopilots = []

    actor_list = world.get_actors()
    vehicles = list(actor_list.filter('vehicle*'))
    for vehicle in vehicles:
        if vehicle.attributes['role_name'] == 'egopilot':
            egopilots.append(vehicle)
    return egopilots


def destroy_all_actors(world):
    """destroy all actors"""
    actor_list = world.get_actors()
    vehicles = list(actor_list.filter('vehicle*'))
    for vehicle in vehicles:
        vehicle.destroy()
    # logger.info('Destroy all vehicles...')

    sensors = list(actor_list.filter('sensor*'))
    for sensor in sensors:
        sensor.destroy()
    # logger.info('Destroy all sensors...')

def destroy(actors):
    try:
        for actor in actors:
            actor.destroy()
    except:
        pass