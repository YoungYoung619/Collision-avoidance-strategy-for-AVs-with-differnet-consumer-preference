import sys, os, glob, random, threading, time, pickle, datetime
from enum import Enum
from situation_assessment import _assess_one_obj_threat_score
from situation_assessment import _score_2_threat_degree
from situation_assessment import comfort_level_scores
from situation_assessment import safety_degree
from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o

import config

try:
    sys.path.append(config.carla_egg_file)
    import carla
except:
    raise ImportError('Please check your carla file')
from carla_utils.logging import logger
from carla_utils.world_ops import *

from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate

class STRATEGY(Enum):
    AGGRESSIVE = 0
    NORMAL = 1
    CONSERVATIVE = 2

########## ------- collision avoidance settings ------ #####
collision_avoidance = True

now = datetime.datetime.now().strftime('%Y%m%d%H%M%S')

experiment_name_map = {STRATEGY.AGGRESSIVE:'strategy_1',
                       STRATEGY.NORMAL:'strategy_2',
                       STRATEGY.CONSERVATIVE:'strategy_3'}

save_experiment_data = False
strategy_type = STRATEGY.CONSERVATIVE
experiment_name = experiment_name_map[strategy_type]
save_experiment_data_to = './experiment_results/' + experiment_name + '/' + experiment_name + '-' + str(now) + '.pickle'
save_experiment_video_to = './experiment_results/' + experiment_name + '/' + experiment_name + '-' + str(now) + '.avi'
########## ------- collision avoidance settings ------ #####

def get_host(world):
    # aa = list(world.get_actors())
    actors = world.get_actors().filter('vehicle*')
    for actor in actors:
        if actor.attributes['role_name'] == 'host_vehicle':
            return actor
    ### actor.get_control()
    raise ValueError('no host in world')

def get_other(world):
    actors = world.get_actors().filter('vehicle*')
    for actor in actors:
        if actor.attributes['role_name'] == 'other_vehicle':
            return actor
    raise ValueError('no other in world')

def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial

    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order + 1)
    half_window = (window_size - 1) // 2
    # precompute coefficients
    b = np.mat([[k ** i for i in order_range] for k in range(-half_window, half_window + 1)])
    m = np.linalg.pinv(b).A[deriv] * rate ** deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs(y[1:half_window + 1][::-1] - y[0])
    lastvals = y[-1] + np.abs(y[-half_window - 1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve(m[::-1], y, mode='valid')

def plot_threat_curve(thread_record_d, thread_record_a, thread_record_s):
    title_font_dict = {'family': 'Times New Roman',
                       'color':  'black',
                       'weight': 'normal',
                       'size': 15}

    axis_font_dict = {'family': 'Times New Roman',
                       'color':  'black',
                       'weight': 'normal',
                       'size': 12}
    legend_font = {'family': 'Times New Roman',
                   'weight': 'normal',
                   'size': 12,
             }

    thread_record_s = np.array(thread_record_s)
    thread_record_a = np.array(thread_record_a)
    thread_record_d = np.array(thread_record_d)
    end = thread_record_s[:, 0][len(thread_record_s[:, 0]) - 1]

    step = 4
    thread_record_s = thread_record_s[0:int(end):step]
    thread_record_a = thread_record_a[0:int(end):step]
    thread_record_d = thread_record_d[0:int(end):step]

    # plt.plot(thread_record_s[:, 0] / 10., thread_record_s[:, 1], color="green")
    # plt.plot(thread_record_a[:, 0] / 10., thread_record_a[:, 1], color="orange")
    # plt.plot(thread_record_d[:, 0] / 10., thread_record_d[:, 1], color="red")
    # plt.ylim(0, 1.1)
    # plt.show()
    xnew = np.linspace(0, (end - step - 1) / 10, 200)
    func_s = interpolate.interp1d(thread_record_s[:, 0] / 10., thread_record_s[:, 1], kind='slinear')
    func_a = interpolate.interp1d(thread_record_a[:, 0] / 10., thread_record_a[:, 1], kind='slinear')
    func_d = interpolate.interp1d(thread_record_d[:, 0] / 10., thread_record_d[:, 1], kind='slinear')

    ynew_s = func_s(xnew)
    ynew_a = func_a(xnew)
    ynew_d = func_d(xnew)

    # func_s_ = interpolate.interp1d(xnew, ynew_s, kind='cubic')
    # func_a_ = interpolate.interp1d(xnew, ynew_a, kind='cubic')
    # func_d_ = interpolate.interp1d(xnew, ynew_d, kind='cubic')
    #
    # xnew = np.arange(0, end/10, 0.01)
    # ynew_s = func_s_(xnew)
    # ynew_a = func_a_(xnew)
    # ynew_d = func_d_(xnew)

    # plt.plot(xnew, ynew_s, color="green")
    # plt.plot(xnew, ynew_a, color="orange")
    # plt.plot(xnew, ynew_d, color="red")
    # plt.ylim(0, 1.1)
    # plt.show()

    ynew_s = savitzky_golay(ynew_s, 21, order=2)
    ynew_a = savitzky_golay(ynew_a, 21, order=2)
    ynew_d = savitzky_golay(ynew_d, 21, order=2)

    # plt.figure(figsize=(16, 7))
    fig = plt.figure(figsize=(9, 2.35), dpi=150)
    plt.plot(xnew, np.clip(ynew_s, 0., 1.), color="green", label="Safe", linewidth=2)
    plt.fill_between(xnew, 0, np.clip(ynew_s, 0., 1.), alpha=0.2, facecolor='green')

    plt.plot(xnew, np.clip(ynew_a, 0., 1.), color="orange", label="Attentive", linewidth=2)
    plt.fill_between(xnew, 0, np.clip(ynew_a, 0., 1.), alpha=0.2, facecolor='orange')

    plt.plot(xnew, np.clip(ynew_d, 0., 1.), color="red", label="Dangerous", linewidth=2)
    plt.fill_between(xnew, 0, np.clip(ynew_d, 0., 1.), alpha=0.2, facecolor='red')

    plt.title('Likelihood of Different Threat Degree', fontdict=title_font_dict)
    plt.xlabel('Time (s)', fontdict=axis_font_dict)
    plt.ylabel('Likelihood', fontdict=axis_font_dict)
    plt.tick_params(labelsize=12)
    plt.legend(prop=legend_font)
    plt.ylim(0., 1.)
    left, right = plt.xlim()
    plt.xlim(0., right)
    plt.show()
    pass


def plot_comfort_curve(comfort_level_m, comfort_level_a, comfort_level_c):
    title_font_dict = {'family': 'Times New Roman',
                       'color':  'black',
                       'weight': 'normal',
                       'size': 15}

    axis_font_dict = {'family': 'Times New Roman',
                       'color':  'black',
                       'weight': 'normal',
                       'size': 12}
    legend_font = {'family': 'Times New Roman',
                   'weight': 'normal',
                   'size': 12,
             }

    comfort_level_c = np.array(comfort_level_c)
    comfort_level_a = np.array(comfort_level_a)
    comfort_level_m = np.array(comfort_level_m)
    end = comfort_level_c[:, 0][len(comfort_level_c[:, 0]) - 1]

    step = 4
    comfort_level_c = comfort_level_c[0:int(end):step]
    comfort_level_a = comfort_level_a[0:int(end):step]
    comfort_level_m = comfort_level_m[0:int(end):step]

    # plt.plot(thread_record_s[:, 0] / 10., thread_record_s[:, 1], color="green")
    # plt.plot(thread_record_a[:, 0] / 10., thread_record_a[:, 1], color="orange")
    # plt.plot(thread_record_d[:, 0] / 10., thread_record_d[:, 1], color="red")
    # plt.ylim(0, 1.1)
    # plt.show()
    xnew = np.linspace(0, (end - step - 1) / 10, 200)
    func_s = interpolate.interp1d(comfort_level_c[:, 0] / 10., comfort_level_c[:, 1], kind='slinear')
    func_a = interpolate.interp1d(comfort_level_a[:, 0] / 10., comfort_level_a[:, 1], kind='slinear')
    func_d = interpolate.interp1d(comfort_level_m[:, 0] / 10., comfort_level_m[:, 1], kind='slinear')

    ynew_s = func_s(xnew)
    ynew_a = func_a(xnew)
    ynew_d = func_d(xnew)

    # func_s_ = interpolate.interp1d(xnew, ynew_s, kind='cubic')
    # func_a_ = interpolate.interp1d(xnew, ynew_a, kind='cubic')
    # func_d_ = interpolate.interp1d(xnew, ynew_d, kind='cubic')
    #
    # xnew = np.arange(0, end/10, 0.01)
    # ynew_s = func_s_(xnew)
    # ynew_a = func_a_(xnew)
    # ynew_d = func_d_(xnew)

    # plt.plot(xnew, ynew_s, color="green")
    # plt.plot(xnew, ynew_a, color="orange")
    # plt.plot(xnew, ynew_d, color="red")
    # plt.ylim(0, 1.1)
    # plt.show()

    ynew_s = savitzky_golay(ynew_s, 21, order=2)
    ynew_a = savitzky_golay(ynew_a, 21, order=2)
    ynew_d = savitzky_golay(ynew_d, 21, order=2)

    # plt.figure(figsize=(16, 7))
    plt.figure(figsize=(9, 2.35), dpi=150)
    plt.plot(xnew, np.clip(ynew_s, 0., 1.), color="green", label="Comfortable", linewidth=2)
    plt.fill_between(xnew, 0, np.clip(ynew_s, 0., 1.), alpha=0.2, facecolor='green')

    plt.plot(xnew, np.clip(ynew_a, 0., 1.), color="orange", label="Moderate", linewidth=2)
    plt.fill_between(xnew, 0, np.clip(ynew_a, 0., 1.), alpha=0.2, facecolor='orange')

    plt.plot(xnew, np.clip(ynew_d, 0., 1.), color="red", label="Radical", linewidth=2)
    plt.fill_between(xnew, 0, np.clip(ynew_d, 0., 1.), alpha=0.2, facecolor='red')

    plt.title('Passenger Comfort Assessment', fontdict=title_font_dict)
    plt.xlabel('Time (s)', fontdict=axis_font_dict)
    plt.ylabel('Score', fontdict=axis_font_dict)
    plt.tick_params(labelsize=12)
    plt.legend(prop=legend_font)
    plt.ylim(0., 1.)
    left, right = plt.xlim()
    plt.xlim(0., right)
    plt.show()
    pass


def plot_vel_acc_rdis(vel, acc, rdis):
    title_font_dict = {'family': 'Times New Roman',
                       'color': 'black',
                       'weight': 'normal',
                       'fontsize': 15}

    axis_font_dict = {'family': 'Times New Roman',
                      'color': 'black',
                      'weight': 'normal',
                      'fontsize': 30}
    legend_font = {'family': 'Times New Roman',
                   'weight': 'normal',
                   'size': 10,
                   }
    vel_x = np.array(vel)[:, 0]/10.
    vel_y = np.array(vel)[:, 1]

    acc_x = np.array(acc)[:, 0]/10.
    acc_y = np.array(acc)[:, 1]

    rdis_x = np.array(rdis)[:, 0]/10.
    rdis_y = np.array(rdis)[:, 1]


    fig  = plt.figure(figsize=(13, 3.5), dpi=120)

    vel_axes = HostAxes(fig, [0.05, 0.1, 0.8, 0.8])
    acc_axes = ParasiteAxes(vel_axes, sharex=vel_axes)
    rdis_axes = ParasiteAxes(vel_axes, sharex=vel_axes)
    vel_axes.parasites.append(acc_axes)
    vel_axes.parasites.append(rdis_axes)

    vel_axes.set_ylabel('Velocity (m/s)')
    vel_axes.set_xlabel('Time (s)')

    vel_axes.axis['right'].set_visible(False)
    acc_axes.axis['right'].set_visible(True)

    acc_axes.set_ylabel('Acceleration (m/s^2)')

    acc_axes.axis['right'].major_ticklabels.set_visible(True)
    acc_axes.axis['right'].label.set_visible(True)

    rdis_axes.set_ylabel('Relative distance (m)')
    offset = (50, 0)
    new_axisline = rdis_axes._grid_helper.new_fixed_axis
    rdis_axes.axis['right2'] = new_axisline(loc='right', axes=rdis_axes, offset=offset)

    fig.add_axes(vel_axes)

    p1, = vel_axes.plot(vel_x, vel_y, label="Vel", linewidth=2)
    p2, = acc_axes.plot(acc_x, acc_y, label="Acc", linewidth=2)
    p3, = rdis_axes.plot(rdis_x, rdis_y, label="RD", linewidth=2)

    vel_axes.legend(prop=legend_font)

    vel_axes.set_ylim(0, np.max(vel_y)+1)
    vel_axes.set_xlim(0, np.max(vel_x))
    rdis_axes.set_ylim(0, np.max(rdis_y)+1)

    # plt.plot(vel_x, vel_y, color="green", label="Velocity", linewidth=2)
    # plt.ylabel('Velocity (m/s)', fontdict=axis_font_dict)
    #
    # plt.plot(acc_x, acc_y, color="orange", label="Acceleration", linewidth=2, secondary_y=True)
    # plt.ylabel('Acceleration (m/s^2)', fontdict=axis_font_dict)
    #

    plt.title('Kinematic information of host vehicle', fontdict=title_font_dict)
    # plt.xlabel('Time (s)', fontdict=axis_font_dict)
    # plt.ylabel('Likelihood', fontdict=axis_font_dict)
    # plt.tick_params(labelsize=20)
    plt.legend(prop=legend_font)

    # plt.ylim(0., 1.)
    # left, right = plt.xlim()
    # plt.xlim(0., right)
    plt.show()
    pass

def control_host(host_vehicle):
    """ a controller control the host, if emergency event(Collision) would be happen, it would prevent it."""
    def pd_control_vel(velocity, target_velocity, last_error, error_sum):
        """a simple PID controller of host vehicle for coliision purpose"""
        k_p = 1.
        k_d = 0.5
        k_i = 0.05

        error = target_velocity - velocity
        error_sum += error
        d_error = error - last_error

        error_sum = min(error_sum, 20)
        throttle = k_p*error + k_d*d_error + k_i*error_sum
        # print('throttle- ', error_sum)
        return max(min(throttle, 1.), 0.), error, error_sum

    def pd_control_for_safety(degree_likelihood, target_likelihood, last_error, error_sum, kp, kd, ki):
        """aa
        Return:
            if val > 0, brake,
            else, throttle
        """
        # k_p = 4
        # k_d = 2.5

        k_p = kp
        k_d = kd
        k_i = ki

        error = target_likelihood - degree_likelihood
        error_sum += error
        d_error = error - last_error

        val = k_p*error + k_d*d_error + k_i*error_sum

        return max(min(val, 1.), -1.), error, error_sum
    global stop
    last_error_pdcc = 0.
    error_sum_pdcc = 0.
    last_error_pdcs = 0.
    error_sum_pdcs = 0.
    time_step = 0
    emergency_control = False
    normal_drive = True
    thread_record_d = []
    thread_record_a = []
    thread_record_s = []

    comfort_record_m = []
    comfort_record_a = []
    comfort_record_c = []

    vel_record = []
    acc_record = []
    relative_distance_record = []
    other = get_other(world)
    record_start_step = 10
    while True:
        ## record info
        pos = host_vehicle.get_location()
        velocity = host_vehicle.get_velocity()
        velocity_ = math.sqrt(velocity.x**2 + velocity.y**2)
        # print('host, ', velocity_)
        # vel_record.append([time_step, velocity_])
        acc = host_vehicle.get_acceleration()
        # acc_record.append([time_step, -acc.x])

        if time_step >= record_start_step:
            vel_record.append([time_step - record_start_step, velocity_])
            acc_record.append([time_step - record_start_step, acc.x])

        comfort_scores = comfort_level_scores(acc.x)
        if time_step >= record_start_step:
            comfort_record_m.append([time_step - record_start_step, comfort_scores[0]])
            comfort_record_a.append([time_step - record_start_step, comfort_scores[1]])
            comfort_record_c.append([time_step - record_start_step, comfort_scores[2]])

        other_pos = other.get_location()
        other_velocity = other.get_velocity()

        if time_step >= record_start_step:
            relative_distance_record.append([time_step - record_start_step, pos.distance(other_pos)])

        ## assess the situation
        ego_v_state = ego_v.ego_vehicle()
        ego_v_state.set_position(position=(pos.x, pos.y, pos.z))
        ego_v_state.set_linear(linear=(velocity.x, velocity.y, velocity.z))
        ego_v_state.set_size(size=(2.5, 1.6, 1.6))

        road_obj_state = road_o.road_obj()
        road_obj_state.set_position(position=(other_pos.x, other_pos.y, other_pos.z))
        road_obj_state.set_linear(linear=(other_velocity.x, other_velocity.y, other_velocity.z))
        road_obj_state.set_size(size=(2.5, 1.6, 1.6))
        score, ttc_test = _assess_one_obj_threat_score(ego_v_state, road_obj_state)
        degree = _score_2_threat_degree(score)
        print(str(score) + '---'+str(degree))
        if time_step >= record_start_step:
            thread_record_d.append([time_step - record_start_step, score[0]])
            thread_record_a.append([time_step - record_start_step, score[1]])
            thread_record_s.append([time_step - record_start_step, score[2]])

        if normal_drive:
            throttle, last_error_pdcc, error_sum_pdcc= pd_control_vel(velocity_, config.host_target_vel_2,
                                                                      last_error_pdcc, error_sum_pdcc)
            control = carla.VehicleControl(throttle=throttle)
            host_vehicle.apply_control(control)

        if emergency_control:
            # ## strategy-1 : emergency braking
            if strategy_type == STRATEGY.AGGRESSIVE:
                if degree == safety_degree.dangerous and score[0] >= 0.5:
                    print('brake!!')
                    control = carla.VehicleControl(brake=1.)
                    host_vehicle.apply_control(control)
                elif degree == safety_degree.safe and score[2] >= 0.9:
                    control = carla.VehicleControl(throttle=0.35)
                    host_vehicle.apply_control(control)
                else:
                    control = carla.VehicleControl(throttle=0., brake=1.)
                    host_vehicle.apply_control(control)
            ## strategy-1 : emergency braking

            ## strategy-2 : pd control
            elif strategy_type == STRATEGY.CONSERVATIVE:
                val, last_error_pdcs, error_sum_pdcs = pd_control_for_safety(score[2], 0.8, last_error_pdcs,
                                                                             error_sum_pdcs, kp=0.1, kd=0.1, ki=0.)
                if val >= 0:
                    # print(val)
                    control = carla.VehicleControl(brake=val/10.)
                else:
                    control = carla.VehicleControl(throttle=0.3)

            ## strategy-3 : pd control
            elif strategy_type == STRATEGY.NORMAL:
                val, last_error_pdcs, error_sum_pdcs = pd_control_for_safety(score[1], 0.5, last_error_pdcs,
                                                                             error_sum_pdcs, kp=2., kd=0.5, ki=0.)
                if val <= 0:
                    control = carla.VehicleControl(brake=math.fabs(val))
                else:
                    control = carla.VehicleControl(throttle=0.35)
            else:
                raise ValueError('strategy_type wrong...')

            host_vehicle.apply_control(control)

        if pos.x > 55:
            logger.info('Stop test...')
            stop = True
            break

        ## transform the control state
        ## for strategy-1
        if strategy_type == STRATEGY.AGGRESSIVE:
            if collision_avoidance and degree == safety_degree.dangerous and score[0] >= 0.5:
                pd_control = False
                emergency_control = True

        ## for strategy-2
        elif  strategy_type == STRATEGY.CONSERVATIVE:
            if collision_avoidance and degree == safety_degree.safe and score[2] <= 0.8:
                pd_control = False
                emergency_control = True

        ## for strategy-3
        elif strategy_type == STRATEGY.NORMAL:
            if collision_avoidance and degree == safety_degree.attentive and score[1] >= 0.65:
                pd_control = False
                emergency_control = True

        #
        #
        # # logger.info('threat degree for other vehicle:'+str(degree))
        # ## assess the situation
        #
        # ## control
        # if collision_avoidance:
        #     if not emergency_control:
        #         throttle, last_error = pd_control_for_collision(velocity, distance_collision, last_error)
        #         control = carla.VehicleControl(throttle=throttle)
        #     else:
        #         control = carla.VehicleControl(brake=1.)
        #         if velocity_ < 0.01:
        #             logger.info('Stop testing...')
        #             i = 0
        #             while (i<20):
        #                 thread_record_d.append([time_step, 0])
        #                 thread_record_a.append([time_step, 0])
        #                 thread_record_s.append([time_step, 1])
        #
        #                 comfort_record_m.append([time_step, 0])
        #                 comfort_record_a.append([time_step, 0])
        #                 comfort_record_c.append([time_step, 1])
        #
        #                 vel_record.append([time_step, velocity_])
        #                 acc_record.append([time_step, -acc.x])
        #
        #                 pos = get_host(world).get_location()
        #                 other_pos = get_other(world).get_location()
        #                 relative_distance_record.append([time_step, pos.distance(other_pos)])
        #
        #                 time_step += 1
        #                 i += 1
        #                 time.sleep(0.1)
        #             break
        # else:
        #     throttle, last_error = pd_control_for_collision(velocity, distance_collision, last_error)
        #     control = carla.VehicleControl(throttle=throttle)
        # host_vehicle.apply_control(control)


        # if distance_collision > 10:
        #     control = carla.VehicleControl(throttle=1.)
        #     host_vehicle.apply_control(control)
        # else:
        #     control = carla.VehicleControl(brake=1.)
        #     host_vehicle.apply_control(control)
        time.sleep(0.05)
        time_step += 1

    if save_experiment_data:
        plot_threat_curve(thread_record_d, thread_record_a, thread_record_s)
        plot_vel_acc_rdis(vel_record, acc_record, relative_distance_record)
        plot_comfort_curve(comfort_record_m, comfort_record_a, comfort_record_c)
        save_dict = {'experiment_name': experiment_name,
                     'thread_record_d': thread_record_d,
                     'thread_record_a': thread_record_a,
                     'thread_record_s': thread_record_s,
                     'vel_record': vel_record,
                     'acc_record': acc_record,
                     'relative_distance_record': relative_distance_record,
                     'comfort_record_m': comfort_record_m,
                     'comfort_record_a': comfort_record_a,
                     'comfort_record_c': comfort_record_c}
        with open(save_experiment_data_to, 'wb') as f:
            pickle.dump(save_dict, f)
            logger.info('save success... file - %s'%(save_experiment_data_to))


def control_other(other_vehicle):
    def pd_control_vel(velocity, target_velocity, last_error, error_sum):
        """a simple PD controller of host vehicle for coliision purpose"""
        k_p = 1.
        k_d = 0.5
        k_i = 0.05

        error = target_velocity - velocity
        d_error = error - last_error
        error_sum += error_sum
        error_sum = min(error_sum, 20)

        throttle = k_p*error + k_d*d_error + k_i*error_sum

        return max(min(throttle, 1.), 0.), error, error_sum

    last_error = 0.
    error_sum = 0.
    while True:
        # ## record info
        # velocity = other_vehicle.get_velocity()
        # acc = other_vehicle.get_acceleration()
        # logger.info('other_vehicle velocity:' + str(velocity) + ' acc:' + str(acc))

        # pos = other_vehicle.get_location()
        # distance_collision = pos.distance(carla.Location(x=-77.5, y=-3., z=pos.z))
        # logger.info('remaining distance:' + str(distance_collision))

        other_vehicle_vel = other_vehicle.get_velocity()
        other_vehicle_vel_ = math.sqrt(other_vehicle_vel.x**2 + other_vehicle_vel.y**2)
        # print('other, ', other_vehicle_vel_)
        throttle, last_error, error_sum = pd_control_vel(other_vehicle_vel_, config.other_vehicle_vel_2, last_error, error_sum)
        control = carla.VehicleControl(throttle=throttle)
        other_vehicle.apply_control(control)
        time.sleep(0.07)


if __name__ == '__main__':
    import cv2
    from carla_utils.sensor_ops import bgr_camera

    #### carla world init ####
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)  # seconds
    logger.info('Carla connect success...')

    logger.info('Carla world initing...')
    world = client.get_world()

    destroy_all_actors(world)

    stop = False

    ## vehicle blueprint
    blueprints = world.get_blueprint_library().filter('*tesla')
    blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]

    ## host vehicle settings
    host_vehicle_bp = random.choice(blueprints)
    if host_vehicle_bp.has_attribute('color'):
        color = host_vehicle_bp.get_attribute('color').recommended_values[0]
        host_vehicle_bp.set_attribute('color', color)
        host_vehicle_bp.set_attribute('role_name', 'host_vehicle')
    transform = carla.Transform(carla.Location(x=config.host_vehicle_init_pos_2[0], y=config.host_vehicle_init_pos_2[1], z=1.8), carla.Rotation(pitch=0., yaw=0., roll=0.))
    try_spawn_at(world, host_vehicle_bp, transform, autopilot=False)

    ## other vehicle settings
    other_vehicle_bp = random.choice(blueprints)
    if other_vehicle_bp.has_attribute('color'):
        color = other_vehicle_bp.get_attribute('color').recommended_values[1]
        other_vehicle_bp.set_attribute('color', color)
        other_vehicle_bp.set_attribute('role_name', 'other_vehicle')
    transform = carla.Transform(carla.Location(x=config.host_vehicle_init_pos_2[0]+50, y=config.host_vehicle_init_pos_2[1], z=1.8),
                                carla.Rotation(pitch=0., yaw=0., roll=0.))
    try_spawn_at(world, other_vehicle_bp, transform, autopilot=False)

    camera_config = {'data_type': 'sensor.camera.rgb', 'image_size_x': 600,
                     'image_size_y': 400, 'fov': 110, 'sensor_tick': 0.02,
                     'transform': carla.Transform(carla.Location(x=-0., y=-0.4, z=1.25)),
                     'attach_to': get_host(world)}

    camera = bgr_camera(world, camera_config)

    time.sleep(1)   ## waiting carla synchronous

    control_host_t = threading.Thread(target=control_host, args=(get_host(world),))
    control_other_t = threading.Thread(target=control_other, args=(get_other(world),))
    control_host_t.start()
    control_other_t.start()

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    VideoWriter = cv2.VideoWriter(save_experiment_video_to, fourcc, 25, (600, 400))

    while True:
        bgr = camera.get()
        #
        # cv2.imshow('test', bgr)
        # cv2.waitKey(1)
        #
        VideoWriter.write(bgr)
        time.sleep(1. / 25.)

        if stop:
            destroy_all_actors(world)
            cv2.destroyAllWindows()
            break
        pass

        pass