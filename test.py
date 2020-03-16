# import time
# from obj_state.road_obj import *
# import numpy as np
# import matplotlib.pyplot as plt
# import math
# import mpl_toolkits.mplot3d
#
#
# # safety degree #
# @unique
# class safety_degree(Enum):
#     """describ the safety degree
#     """
#     safe = 0
#     attentive = 1
#     dangerous = 2
#
# @unique
# class relation_state(Enum):
#     """describ the relation between ego vehicle and other obj
#     Params:
#         none_overlap:
#         one_overlap:
#         all_overlap:
#     """
#     none_overlap = 0
#     overlap_in_x = 1
#     overlap_in_y = 2
#     all_overlap = 3
#
#
# if __name__ == "__main__":
#     """obj_state.road_obj test"""
#     # ## init some vars ##
#     # size = (1., 2., 3.)
#     # position = (2., 3., 4.)
#     # orientation = (1., 0., 0., 0.)
#     # linear = (1., 2., 3.)
#     # angular = 3. ## default in axis z
#     # t = time.time()
#     # obj_type = obj_type.vehicle
#     # obj = road_obj()
#     #
#     # ## set attr ##
#     # obj.set_size(size)
#     # obj.set_position(position)
#     # obj.set_linear(linear)
#     # obj.set_orientation(orientation)
#     # obj.set_angular(angular)
#     # obj.set_time_stamp(t)
#     # obj.set_obj_type(obj_type)
#     #
#     # ## get attr ##
#     # s = obj.get_size()
#     #
#     # pt = obj.get_position()
#     # l = obj.get_linear()
#     # ps1 = obj.get_orientation(format = 'rpy')
#     # ps2 = obj.get_orientation(format = 'xyzw')
#     # a = obj.get_angular()
#     # t = obj.get_time_stamp()
#     # type = obj.get_type()
#
#     """2d Gauss test"""
#     # x, y = np.mgrid[-2:2:200j, -2:2:200j]
#     # z = (1 / 2 * math.pi * 3 ** 2) * np.exp(-(x ** 2 + y ** 2) / 2 * 3 ** 2)
#     # ax = plt.subplot(111, projection='3d')
#     # ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='rainbow',
#     #                 alpha=0.9)  # 绘面
#     # ax.set_xlabel('x')
#     # ax.set_ylabel('y')
#     # ax.set_zlabel('z')
#     # plt.show()
#
#
#     # """plot the gaussian"""
#     # def gaussian_1d(x, mean, for_what, std=.5):
#     #     """produce a val respresents the socre according the gaussian distribution.
#     #     Args:
#     #         x: a input val.
#     #         mean: the mean of gaussian distribution.
#     #         std: the std of gaussian distribution.
#     #         for_what: should be one of safety_degree
#     #     """
#     #
#     #     def norm(x, mu, sigma):
#     #         """normal gaussian function
#     #         """
#     #         pdf = math.exp(-((x - mu) ** 2) / (2 * sigma ** 2)) / (sigma * math.sqrt(2 * np.pi))
#     #         return 3*pdf
#     #
#     #     assert for_what in list(safety_degree.__members__.values())
#     #
#     #     if for_what is safety_degree.dangerous:
#     #         if x < mean:
#     #             score = norm(x=mean, mu=mean, sigma=std)
#     #         else:
#     #             score = norm(x=x, mu=mean, sigma=std)
#     #     elif for_what is safety_degree.attentive:
#     #         score = norm(x=x, mu=mean, sigma=std)
#     #     elif for_what is safety_degree.safe:
#     #         if x > mean:
#     #             score = norm(x=mean, mu=mean, sigma=std)
#     #         else:
#     #             score = norm(x=x, mu=mean, sigma=std)
#     #     else:
#     #         raise ValueError("Error")
#     #
#     #     return score
#     #
#     # plt.subplot(121)
#     # xs = np.arange(0., 5., 0.01)
#     # y_d = []
#     # for x in xs:
#     #     y_d.append(gaussian_1d(x, mean=1.8, for_what=safety_degree.dangerous))
#     # plt.plot(xs, y_d, label="dangerous", color='red')
#     #
#     # y_a = []
#     # for x in xs:
#     #     y_a.append(gaussian_1d(x, mean=2.16, for_what=safety_degree.attentive))
#     # plt.plot(xs, y_a, label="attentive", color='goldenrod')
#     #
#     # y_s = []
#     # for x in xs:
#     #     y_s.append(gaussian_1d(x, mean=2.88, for_what=safety_degree.safe))
#     # plt.plot(xs, y_s, label="safe", color='green')
#     #
#     # plt.legend()
#     # plt.title('Score')
#     # plt.xlabel('Time_Diff (sec)')
#     # plt.ylabel('Score')
#     # # 输出
#     # #plt.show()
#     #
#     # """softmax"""
#     # import numpy as np
#     # import matplotlib.pyplot as plt
#     #
#     #
#     # def softmax(x):
#     #     return np.exp(x) / np.expand_dims(np.sum(np.exp(x), axis=-1),axis=-1)
#     #
#     #
#     # score = np.stack([y_d, y_a, y_s], axis=-1)
#     # score = softmax(score)
#     # # ax = plt.subplot(122, projection='3d')
#     # # ax.title('After softmax')
#     # # ax.set_xlabel('Time_Diff (sec)')
#     # # ax.set_ylabel('vel')
#     # # ax.set_zlabel('Probability')
#     # # # plt.plot(xs, score[:, 0], label="dangerous", color='red')
#     # # # plt.plot(xs, score[:, 1], label="attentive", color='goldenrod')
#     # # # plt.plot(xs, score[:, 2], label="dangerous", color='green')
#     # # # plt.legend()
#     # # # plt.show()
#     # # ax.plot(x, ys=0, zs=score[:,0], label='dangerous')
#     # # plt.show()
#     # ax = plt.subplot(122, projection='3d')
#     # plt.title("After softmax")
#     # ax.set_xlabel("Time_Diff (sec)")
#     # ax.set_ylabel("Vel_Diff (m/s)")
#     # ax.set_zlabel("P")
#     # ax.plot(xs, ys=np.zeros(shape=xs.shape), zs=score[:, 0], label='dangerous', color='red')
#     # ax.plot(xs, ys=np.zeros(shape=xs.shape), zs=score[:, 1], label='attentive', color='goldenrod')
#     # ax.plot(xs, ys=np.zeros(shape=xs.shape), zs=score[:, 2], label='safe', color='green')
#     # plt.show()
#
#     # ego_pos_vec = np.array([100, 100])
#     # other_pos_vec = np.array([200, 200])
#     # pos_vec = ego_pos_vec - other_pos_vec
#     # distance_2_collision = np.sqrt(np.sum(np.square(pos_vec)))
#     # print(distance_2_collision)
#
#     a = -0.5235
#     r_matix = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
#     p = np.matmul(np.array([1, 0]), r_matix)
#     pass
#
#
#

import numpy as np

aa = np.random.uniform(0, 1, (22, 22, 3))
bb = aa[:5]
pass