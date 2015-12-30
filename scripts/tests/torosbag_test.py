#!/usr/bin/env python

import grid
import rospy
import numpy as np

'''
Create skills
* load demonstrations for each of the different actions
* learn skill representations for each one
'''

# start ROS up
rospy.init_node('create_skills_node')

# configuration for skills
ak = 1
gk = 1

# first set up filenames
approach_filenames = ['data/sim/app1.yml','data/sim/app2.yml','data/sim/app3.yml']
grasp_filenames = ['data/sim/grasp1.yml','data/sim/grasp2.yml','data/sim/grasp3.yml']
transport_filenames = ['data/sim/transport1.yml','data/sim/transport2.yml','data/sim/transport3.yml']
align_filenames = ['data/sim/align1.yml','data/sim/align2.yml','data/sim/align3.yml']
place_filenames = ['data/sim/place1.yml','data/sim/place3.yml']
disengage_filenames = ['data/sim/disengage1.yml','data/sim/disengage2.yml','data/sim/disengage3.yml']
release_filenames = ['data/sim/release1.yml','data/sim/release2.yml','data/sim/release3.yml']

skill_filenames = {}
skill_filenames['approach'] = approach_filenames
skill_filenames['grasp'] = grasp_filenames
skill_filenames['disengage'] = disengage_filenames
skill_filenames['release'] = release_filenames
skill_filenames['align'] = align_filenames
skill_filenames['place'] = place_filenames

skill_objs = {'approach':['time','link'], 'grasp':['time','link'], 'transport':['time','node'], 'align':['time','node'], 'release':['time','node'],'place':['time','node'],'disengage':['time','link']}#, 'init':[]}
skill_fixed = {'approach':[], 'grasp':[], 'transport':['link'], 'align':['link'],'place':['link'],'disengage':[],'release':['link']} #,'init':[]}

all_params = []

name=skill_filenames.keys()[0]
filenames=skill_filenames.values()[0]
data,goals = grid.LoadData(filenames,skill_objs[name] + ['gripper'],manip_objs=skill_fixed[name])

data[0][0].ToRosBag('test.bag')
