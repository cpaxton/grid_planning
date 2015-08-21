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
ak = 2
gk = 1

# first set up filenames
approach_filenames = ['app1.yml','app2.yml','app3.yml']
grasp_filenames = ['grasp1.yml','grasp2.yml','grasp3.yml']
transport_filenames = ['transport1.yml','transport2.yml','transport3.yml']
backoff_filenames = ['backoff1.yml','backoff2.yml','backoff3.yml']

skill_filenames = {}
skill_filenames['approach'] = approach_filenames
skill_filenames['grasp'] = grasp_filenames
skill_filenames['transport'] = transport_filenames
skill_filenames['backoff'] = backoff_filenames

skill_objs = {'approach':['time','link'], 'grasp':['time','link'], 'transport':['time','link','node'], 'backoff':['time','link']}
skill_fixed = {'approach':[], 'grasp':['link'], 'transport':['link'], 'backoff':[]}

# load data for each skill
for name,filenames in skill_filenames.items():

    data,params,num_weights,goals = grid.LoadDataDMP(filenames,skill_objs[name])
    training_data = np.array(data[0][1])
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i][1]))

    # create the skill object
    skill = grid.RobotSkill(name=name,action_k=ak,goal_k=gk,data=training_data,objs=skill_objs[name],params=params,goals=goals)#,num_weights=num_weights)

    skill.save(name+"_skill.yml")

print "... Done creating skills."

try:
    while not rospy.is_shutdown():
        pass
except rospy.ROSInterruptException,e:
    pass
