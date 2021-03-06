"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :


Author：Team Li
"""

import pickle
from scenario_2.scenario_generator import *

# file_exp = './experiment_results/strategy_1/strategy_1-20200123211723.pickle' ##exp_1 aggressive
file_exp = './experiment_results/strategy_2/strategy_2-20200123213302.pickle' ##exp_1 conservative
# file_exp = './experiment_results/strategy_3/strategy_3-20200123211439.pickle' ## exp_3 normal

with open(file_exp, 'rb') as f:
    dict = pickle.load(f)

    thread_record_d = dict['thread_record_d']
    thread_record_a = dict['thread_record_a']
    thread_record_s = dict['thread_record_s']

    vel_record = dict['vel_record']
    acc_record = dict['acc_record']
    relative_distance_record = dict['relative_distance_record']

    comfort_record_m = dict['comfort_record_m']
    comfort_record_a = dict['comfort_record_a']
    comfort_record_c = dict['comfort_record_c']

    thread_record_d = thread_record_d[:300]
    thread_record_a = thread_record_a[:300]
    thread_record_s = thread_record_s[:300]

    vel_record = vel_record[:300]
    acc_record = acc_record[:300]
    relative_distance_record = relative_distance_record[:300]

    comfort_record_m = comfort_record_m[:300]
    comfort_record_a = comfort_record_a[:300]
    comfort_record_c = comfort_record_c[:300]

    plot_threat_curve(thread_record_d, thread_record_a, thread_record_s)
    plot_vel_acc_rdis(vel_record, acc_record, relative_distance_record)
    plot_comfort_curve(comfort_record_m, comfort_record_a, comfort_record_c)
    pass