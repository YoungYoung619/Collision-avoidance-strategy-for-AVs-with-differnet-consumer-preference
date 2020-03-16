"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :


Author：Team Li
"""
import msgs.scene_msg as scene_m
from situation_assessment import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

weather_title = {scene_m.weather.clear: "clear",
                 scene_m.weather.dust: "dust",
                 scene_m.weather.fog: "fog",
                 scene_m.weather.rain: "rain",}
road_title = {scene_m.road_state.normal: "normal",
              scene_m.road_state.leaves: "leaves",
              scene_m.road_state.wetness: "wetness",
              scene_m.road_state.snow: "snow"}

def softmax(x):
    return np.exp(x) / np.expand_dims(np.sum(np.exp(x), axis=-1), axis=-1)


def plot_distribution_each_degree_2d(vel_km_h, weather_type, road_state, show=True):
    """
    Example:
        road_state = scene_m.road_state.snow
        weather_type = scene_m.weather.fog
        vel_km_h = 100

        plot_distribution_each_degree_2d(vel_km_h=vel_km_h,
                                        weather_type=weather_type,
                                        road_state=road_state ,show=True)
    """
    f = 15
    d = 1.2

    assert weather_type in list(scene_m.weather.__members__.values())
    assert road_state in list(scene_m.road_state.__members__.values())

    vel_m_s = vel_km_h/3.6
    thresh = [(vel_m_s / (MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
               (vel_m_s / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                (vel_m_s / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type])]

    xs = np.arange(0., thresh[2], 0.01)
    y_d = []
    for x in xs:
        y_d.append(gaussian_1d(x, mean=thresh[0], for_what=safety_degree.dangerous,
                               std=(vel_m_s*3.6)/f, max=(vel_m_s*3.6)/d))

    y_a = []
    for x in xs:
        y_a.append(gaussian_1d(x, mean=thresh[1], for_what=safety_degree.attentive,
                               std=(vel_m_s*3.6)/f, max=(vel_m_s*3.6)/d))

    y_s = []
    for x in xs:
        y_s.append(gaussian_1d(x, mean=thresh[2], for_what=safety_degree.safe,
                               std=(vel_m_s*3.6)/f, max=(vel_m_s*3.6)/d))

    score = np.stack([y_d, y_a, y_s], axis=-1)
    probability = softmax(score)


    if show:
        plt.figure()
        plt.suptitle("Velocity:%skm/h Weather:%s Road:%s" % (str(vel_km_h),
                                                             weather_title[weather_type],
                                                             road_title[road_state]))
        plt.subplot(121)
        plt.plot(xs, y_d, label="dangerous", color='red')
        plt.plot(xs, y_a, label="attentive", color='goldenrod')
        plt.plot(xs, y_s, label="safe", color='green')
        plt.legend()
        plt.title('Score')
        plt.xlabel('TTC (sec)')
        plt.ylabel('Score')

        plt.subplot(122)
        plt.plot(xs, probability[:, 0], label="dangerous", color='red')
        plt.plot(xs, probability[:, 1], label="attentive", color='goldenrod')
        plt.plot(xs, probability[:, 2], label="safe", color='green')
        plt.legend()
        plt.title('After Softmax')
        plt.xlabel('TTC (sec)')
        plt.ylabel('Probability')
        plt.show()
    else:
        return xs, probability


def plot_distribution_each_degree_3d(xs, probability, vel_km_h, axes):
    """plot the distribution in each safety degree
    Args:
        xs: the time diff in sec
        probability: the probability in one safety degree
        vel_km_h: velocity in km/h
    Example:
        road_state = scene_m.road_state.normal
        weather_type = scene_m.weather.clear

        plt.suptitle("Weather:%s Road:%s" % (weather_title[weather_type], road_title[road_state]))
        ax = plt.subplot(131, projection='3d')
        plt.title("Dangerous Distribution")
        ax.set_xlabel("TTC (sec)")
        ax.set_ylabel("Velocity (km/h)")
        ax.set_zlabel("Probability")

        ax1 = plt.subplot(132, projection='3d')
        plt.title("Attentive Distribution")
        ax1.set_xlabel("TTC (sec)")
        ax1.set_ylabel("Velocity (km/h)")
        ax1.set_zlabel("Probability")

        ax2 = plt.subplot(133, projection='3d')
        plt.title("Dangerous Distribution")
        ax2.set_xlabel("TTC (sec)")
        ax2.set_ylabel("Velocity (km/h)")
        ax2.set_zlabel("Probability")


        for vel_km_h in range(170):
            xs, score = plot_distribution_each_degree_2d(vel_km_h=vel_km_h,
                                                    weather_type=weather_type, road_state=road_state ,show=False)
            plot_distribution_each_degree_3d(xs, score, vel_km_h, [ax, ax1, ax2])
        plt.show()
    """

    ys = np.zeros(shape=xs.shape)
    ys.fill(float(vel_km_h))
    axes[0].plot(xs, ys=ys, zs=probability[:, 0], label='dangerous', color='red')
    axes[1].plot(xs, ys=ys, zs=probability[:, 1], label='attentive', color='goldenrod')
    axes[2].plot(xs, ys=ys, zs=probability[:, 2], label='safe', color='green')


def plot_surface_3d(weather_type, road_state):
    """
    Example:
        road_state = scene_m.road_state.normal
        weather_type = scene_m.weather.clear

        plot_surface_3d(weather_type, road_state)
    """
    assert weather_type in list(scene_m.weather.__members__.values())
    assert road_state in list(scene_m.road_state.__members__.values())

    f = 15
    d = 1.2

    x_l = [[], [], []]
    y_l = [[], [], []]
    z_l = [[], [], []]
    for vel_km_h in range(170):
        vel_m_s = vel_km_h+1 / 3.6
        thresh = [(vel_m_s / (MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                   (vel_m_s / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                    (vel_m_s / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type])]

        # xs = np.arange(0., thresh[2], thresh[2]/200.+1e-5) ## TTC
        xs = np.arange(0., 50, 50 / 200. + 1e-5)  ## TTC
        y_d = []
        for x in xs:
            y_d.append(gaussian_1d(x, mean=thresh[0], for_what=safety_degree.dangerous,
                                   std=(vel_m_s * 3.6) / f, max=(vel_m_s * 3.6) / d))

        y_a = []
        for x in xs:
            y_a.append(gaussian_1d(x, mean=thresh[1], for_what=safety_degree.attentive,
                                   std=(vel_m_s * 3.6) / f, max=(vel_m_s * 3.6) / d))

        y_s = []
        for x in xs:
            y_s.append(gaussian_1d(x, mean=thresh[2], for_what=safety_degree.safe,
                                   std=(vel_m_s * 3.6) / f, max=(vel_m_s * 3.6) / d))

        score = np.stack([y_d, y_a, y_s], axis=-1)
        probability = softmax(score)

        degree_type = np.argmax(probability, axis=-1)
        degree_p = np.max(probability, axis=-1)

        if len(xs) > 0:
            for index in range(3):
                x_index = []
                y_index = []
                z_index = []
                mask = np.equal(degree_type, index)
                degree_p_ = degree_p*mask
                x_ = xs*mask
                for x__,z__ in zip(x_, degree_p_):
                    if x__!=0. and z__!=0.:
                        x_index.append(x__)
                        y_index.append(vel_km_h)
                        z_index.append(z__)
                x_l[index].append(x_index.copy())
                y_l[index].append(y_index.copy())
                z_l[index].append(z_index.copy())
                pass
        pass
    x_d = np.concatenate(x_l[0])
    x_a = np.concatenate(x_l[1])
    x_s = np.concatenate(x_l[2])

    y_d = np.concatenate(y_l[0])
    y_a = np.concatenate(y_l[1])
    y_s = np.concatenate(y_l[2])

    z_d = np.concatenate(z_l[0])
    z_a = np.concatenate(z_l[1])
    z_s = np.concatenate(z_l[2])

    logger.info("Dangerous Points:%s  --  Attentive Points:%s  --  Safe Points:%s"%(str(len(x_d)),
                                                                                    str(len(x_a)),
                                                                                    str(len(x_s))))

    plt.suptitle("Weather:%s Road:%s" % (weather_title[weather_type], road_title[road_state]))
    ax = plt.gca(projection='3d')
    plt.title("Threat Distribution")
    ax.set_xlabel("TTC (sec)")
    ax.set_ylabel("Velocity (km/h)")
    ax.set_zlabel("Probability")

    ax.scatter(x_d, y_d, z_d, color="red", s=1)
    ax.scatter(x_a, y_a, z_a, color="goldenrod", s=1)
    ax.scatter(x_s, y_s, z_s, color="green", s=1)
    plt.show()
    pass


def plot_surface_2d(weather_type, road_state):
    """
        Example:
            road_state = scene_m.road_state.normal
            weather_type = scene_m.weather.clear

            plot_surface_2d(weather_type, road_state)
        """
    assert weather_type in list(scene_m.weather.__members__.values())
    assert road_state in list(scene_m.road_state.__members__.values())

    f = 15
    d = 1.2

    x_l = [[], [], []]
    y_l = [[], [], []]
    for vel_km_h in range(170):
        vel_m_s = (vel_km_h + 1) / 3.6
        thresh = [(vel_m_s / (MAX_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                  (vel_m_s / (AVG_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[weather_type]),
                  (vel_m_s / (COMFORTABLE_DECELERATION * road_state_ratio[road_state]) * weather_effect_ratio[
                      weather_type])]

        #xs = np.arange(0., thresh[2], thresh[2]/200.+1e-5) ## TTC
        xs = np.arange(0., 40, 40 / 1000.)  ## TTC
        y_d = []
        for x in xs:
            y_d.append(gaussian_1d(x, mean=thresh[0], for_what=safety_degree.dangerous,
                                   std=(vel_m_s * 3.6) / f, max=(vel_m_s * 3.6) / d))

        y_a = []
        for x in xs:
            y_a.append(gaussian_1d(x, mean=thresh[1], for_what=safety_degree.attentive,
                                   std=(vel_m_s * 3.6) / f, max=(vel_m_s * 3.6) / d))

        y_s = []
        for x in xs:
            y_s.append(gaussian_1d(x, mean=thresh[2], for_what=safety_degree.safe,
                                   std=(vel_m_s * 3.6) / f, max=(vel_m_s * 3.6) / d))

        score = np.stack([y_d, y_a, y_s], axis=-1)
        probability = softmax(score)

        degree_type = np.argmax(probability, axis=-1)

        if len(xs) > 0:
            for index in range(3):
                x_index = []
                y_index = []
                mask = np.equal(degree_type, index)
                x_ = xs * mask
                for x__ in x_:
                    if x__ != 0.:
                        x_index.append(x__)
                        y_index.append(vel_km_h)
                x_l[index].append(x_index.copy())
                y_l[index].append(y_index.copy())

    x_d = np.concatenate(x_l[0])
    x_a = np.concatenate(x_l[1])
    x_s = np.concatenate(x_l[2])

    y_d = np.concatenate(y_l[0])
    y_a = np.concatenate(y_l[1])
    y_s = np.concatenate(y_l[2])

    logger.info("Dangerous Points:%s  --  Attentive Points:%s  --  Safe Points:%s" % (str(len(x_d)),
                                                                                          str(len(x_a)),
                                                                                          str(len(x_s))))

    cover_d = round((len(x_d) / (len(x_d) + len(x_a) + len(x_s)))*100, 2)
    cover_a = round((len(x_a) / (len(x_d) + len(x_a) + len(x_s)))*100, 2)
    cover_s = round((len(x_s) / (len(x_d) + len(x_a) + len(x_s)))*100, 2)

    plt.suptitle("Weather:%s ---- Road:%s" % (weather_title[weather_type], road_title[road_state]))
    plt.title("Dangerous:{}% -- Attentive:{}% -- Safe:{}%".format(str(cover_d), str(cover_a), str(cover_s)))
    plt.xlabel("TTC(sec)")
    plt.ylabel("Velocity(km/h)")
    plt.scatter(x_d, y_d, s=20, c="red", label="Dangerous")
    plt.scatter(x_a, y_a, s=20, c="goldenrod", label="Attentive")
    plt.scatter(x_s, y_s, s=20, c="green", label="Safe")
    plt.legend()

    plt.show()



if __name__ == '__main__':
    # road_state = scene_m.road_state.normal
    # weather_type = scene_m.weather.clear
    #
    # plt.suptitle("Weather:%s Road:%s" % (weather_title[weather_type], road_title[road_state]))
    # ax = plt.subplot(131, projection='3d')
    # plt.title("Dangerous Distribution")
    # ax.set_xlabel("TTC (sec)")
    # ax.set_ylabel("Velocity (km/h)")
    # ax.set_zlabel("Probability")
    #
    # ax1 = plt.subplot(132, projection='3d')
    # plt.title("Attentive Distribution")
    # ax1.set_xlabel("TTC (sec)")
    # ax1.set_ylabel("Velocity (km/h)")
    # ax1.set_zlabel("Probability")
    #
    # ax2 = plt.subplot(133, projection='3d')
    # plt.title("Dangerous Distribution")
    # ax2.set_xlabel("TTC (sec)")
    # ax2.set_ylabel("Velocity (km/h)")
    # ax2.set_zlabel("Probability")
    #
    # for vel_km_h in range(170):
    #     xs, score = plot_distribution_each_degree_2d(vel_km_h=vel_km_h,
    #                                                  weather_type=weather_type, road_state=road_state, show=False)
    #     plot_distribution_each_degree_3d(xs, score, vel_km_h, [ax, ax1, ax2])
    # plt.show()

    road_state = scene_m.road_state.normal
    weather_type = scene_m.weather.clear

    plot_surface_3d(weather_type, road_state)

