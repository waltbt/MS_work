#!/usr/bin/env python

import numpy as np
#import matplotlib.pyplot as plt
#import pandas as pd
import os

import json
from sklearn import neighbors, datasets

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

#from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier



from sklearn.metrics import classification_report, confusion_matrix

properties = []
labels = []

"""
Import datasets
"""

os.chdir('/home/ben/gripper_proj_py/data/comb_data/')


filename = 'IR1_combined_data.txt'
with open(filename, 'r') as filehandle:
    data_set = json.load(filehandle)


#temp = []

for data in data_set:
    properties.append(data[0:3])
    labels.append(data[-1])
properties = np.asarray(properties)
properties = properties.astype(np.float64)

# filename = 'Tabbed_leaf.txt'
# with open(filename, 'r') as filehandle:
#     leaf_data = json.load(filehandle)
#
# for data in leaf_data:
#     properties.append(data[0:3])
#     labels.append('leaf')


properties_train, properties_test, labels_train, labels_test = train_test_split(properties, labels, test_size=0.20)

scaler = StandardScaler()
scaler.fit(properties_train)

properties_train = scaler.transform(properties_train)
properties_test = scaler.transform(properties_test)

mlp = MLPClassifier(hidden_layer_sizes=(5,), max_iter=30, alpha=1e-4,
                    solver='sgd', verbose=10, random_state=1,
                    learning_rate_init=.1)

mlp.fit(properties_train, labels_train)
print("Training set score: %f" % mlp.score(properties_train, labels_train))
print("Test set score: %f" % mlp.score(properties_test, labels_test))

labels_pred = mlp.predict(properties_test)


print(confusion_matrix(labels_test, labels_pred))
print(classification_report(labels_test, labels_pred))


scaler = StandardScaler()
scaler.fit(properties_train)

properties_train = scaler.transform(properties_train)
properties_test = scaler.transform(properties_test)
#
#
# classifier = KNeighborsClassifier(n_neighbors=5)
# classifier.fit(properties_train, labels_train)
#
# labels_pred = classifier.predict(properties_test)
#
#
# print(confusion_matrix(labels_test, labels_pred))
# print(classification_report(labels_test, labels_pred))
