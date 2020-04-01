#!/usr/bin/env python

import numpy as np
import os

#import json
import pickle



class sensor_data_struct():
    def __init__(self, point, IR_data, TOF_data, color_data, button_data):
        self.pt = point
        self.IR = IR_data
        self.TOF = TOF_data
        self.color = color_data
        self.button = button_data



comb_data = []
leaf_set = []
berry_set = []

# os.chdir('/home/ben/gripper_proj_py/data/Old_data/')
os.chdir('/home/ben/gripper_proj_py/data/IR_data/')

#filename = 'Tabbed_Sphere_Large_data_1_19_set_1.txt'
filename = 'Basic_Ovoid_Medium_data_1_15_set_1.txt'
with open(filename, 'r') as filehandle:
    data_set_t1 = pickle.load(filehandle)

#filename = 'Tabbed_Sphere_Large_data_1_19_set_2.txt'
filename = 'Basic_Ovoid_Medium_data_1_15_set_2.txt'
with open(filename, 'r') as filehandle:
    data_set_t2 = pickle.load(filehandle)

#filename = 'Tabbed_Sphere_Large_data_1_19_set_3.txt'
filename = 'Basic_Ovoid_Medium_data_1_15_set_3.txt'
with open(filename, 'r') as filehandle:
    data_set_t3 = pickle.load(filehandle)

#filename = 'Tabbed_Sphere_Large_data_1_19_set_4.txt'
filename = 'Basic_Ovoid_Medium_data_1_15_set_4.txt'
with open(filename, 'r') as filehandle:
    data_set_t4 = pickle.load(filehandle)

filename = 'Basic_Ovoid_Medium_data_1_15_set_5.txt'
with open(filename, 'r') as filehandle:
    data_set_t5 = pickle.load(filehandle)


temp_data_set = data_set_t1 + data_set_t2 + data_set_t3 + data_set_t4 + data_set_t5
for data in temp_data_set:
    if data.button_data == 0:
        berry_set.append(data.IR[0:3] +['success!'])
    else:
        berry_set.append(data.IR[0:3] +['Bad Grip'])


leaf_set =[]
#filename = 'Tabbed_leaf.txt'
filename = 'Basic_leaf.txt'
with open(filename, 'r') as filehandle:
    leaf_data = pickle.load(filehandle)

for data in leaf_data:
    leaf_set.append(data.IR[0:3]+ ['leaf'])

comb_data = berry_set + leaf_set

# for data in data_set:
#     properties.append(data[3:6])
#     labels.append(data[-1])

#print(comb_data)
# with open('Tabbed_Sphere_Large_combined_data.txt', 'w') as filehandle:
#         pickle.dump(comb_data, filehandle)
with open('Basic_Ovoid_Medium_combined_data.txt', 'w') as filehandle:
        pickle.dump(comb_data, filehandle)
