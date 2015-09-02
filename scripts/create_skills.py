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
ak = 3
gk = 1

# first set up filenames
approach_filenames = ['data/app1.yml','data/app2.yml','data/app3.yml']
grasp_filenames = ['data/grasp1.yml','data/grasp2.yml','data/grasp3.yml']
transport_filenames = ['data/transport1.yml','data/transport2.yml','data/transport3.yml']
disengage_filenames = ['data/disengage1.yml','data/disengage2.yml','data/disengage3.yml']

skill_filenames = {}
skill_filenames['approach'] = approach_filenames
skill_filenames['grasp'] = grasp_filenames
skill_filenames['transport'] = transport_filenames
skill_filenames['disengage'] = disengage_filenames

#skill_objs = {'approach':['time','link'], 'grasp':['time','link'], 'transport':['time','link','node'], 'disengage':['time','link']}
skill_objs = {'approach':['time','link'], 'grasp':['time','link'], 'transport':['time','node'], 'disengage':['time','link']}
skill_fixed = {'approach':[], 'grasp':[''], 'transport':['link'], 'disengage':[]}

# load data for each skill
for name,filenames in skill_filenames.items():

    # create some GMMs for gripper stuff too!
    # this is just a convenience thing; they really SHOULD be the same as the ones we're using above
    data,params,num_weights,goals = grid.LoadDataDMP(filenames,skill_objs[name] + ['gripper'],manip_objs=skill_fixed[name])

    training_data = np.array(data[0][1])
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i][1]))

    # create the skill object
    skill = grid.RobotSkill(name=name,action_k=ak,goal_k=gk,data=training_data,objs=(skill_objs[name]+['gripper']),manip_objs=skill_fixed[name],params=params,goals=goals)#,num_weights=num_weights)

    skill.save(name+"_skill.yml")

print "... Done creating skills."

try:
    while not rospy.is_shutdown():
        pass
except rospy.ROSInterruptException,e:
    pass
