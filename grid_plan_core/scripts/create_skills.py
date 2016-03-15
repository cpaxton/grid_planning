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

# load data for each skill
for name,filenames in skill_filenames.items():

    # create some GMMs for gripper stuff too!
    # this is just a convenience thing; they really SHOULD be the same as the ones we're using above
    data,goals = grid.LoadData(filenames,skill_objs[name] + ['gripper'],manip_objs=skill_fixed[name])

    training_data = np.array(data[0][1])
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i][1]))

    # create the skill object
    skill = grid.RobotSkill(name=name,action_k=ak,goal_k=gk,data=training_data,objs=(skill_objs[name]+['gripper']),manip_objs=skill_fixed[name],goals=goals)

    skill.save(name+"_skill.yml")

print "... Done creating skills."

try:
    while not rospy.is_shutdown():
        pass
except rospy.ROSInterruptException,e:
    pass
