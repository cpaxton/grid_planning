#!/usr/bin/env python

import grid
import rospy

import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib as mpl

from sklearn.mixture import GMM

rospy.init_node('visualize_models_etc')

app = grid.RobotSkill(filename="skills/approach_skill.yml")
grasp = grid.RobotSkill(filename="skills/grasp_skill.yml")

'''
==========================================================================
'''
# first set up filenames
approach_filenames = ['data/app1.yml','data/app2.yml','data/app3.yml']
grasp_filenames = ['data/grasp1.yml','data/grasp2.yml','data/grasp3.yml']
transport_filenames = ['data/transport1.yml','data/transport2.yml','data/transport3.yml']
disengage_filenames = ['data/disengage1.yml','data/disengage2.yml','data/disengage3.yml']
#init_filenames = ['data/init11.yml','data/init12.yml','data/init13.yml','data/init14.yml']

skill_filenames = {}
skill_filenames['approach'] = approach_filenames
skill_filenames['grasp'] = grasp_filenames

skill_objs = {'approach':['time','link'], 'grasp':['time','link'], 'transport':['time','node'], 'disengage':['time','link']}#, 'init':[]}
skill_fixed = {'approach':[], 'grasp':[''], 'transport':['link'], 'disengage':[]} #,'init':[]}

all_params = []

examples = []

# load data for each skill
nfig = 1;
for name,filenames in skill_filenames.items():
    if name == 'grasp':
        continue

    # create some GMMs for gripper stuff too!
    # this is just a convenience thing; they really SHOULD be the same as the ones we're using above
    data,params,num_weights,goals = grid.LoadDataDMP(filenames,skill_objs[name],manip_objs=skill_fixed[name])
    training_data = np.array(data[0][1])
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i][1]))
    examples.append((name,training_data,goals))

for name,training_data,goals in examples:
    plt.figure(nfig);
    plt.subplot(1,2,1);
    for i in range(2,training_data.shape[1]):
        plt.plot(training_data[:,0],training_data[:,i]);
    plt.title('action %s'%(name))
    plt.subplot(1,2,2);
    goals_arr = np.array(goals)
    for i in range(goals[0].shape[0]):
        plt.plot(goals_arr[:,i]);
    nfig += 1
    plt.legend(labels=data[0][0].GetFeatureLabels([skill_objs[name][1]]));
    plt.title('end %s'%(name))
plt.show()    

