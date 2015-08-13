#!/usr/bin/env

import grid
import rospy

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
approach_filenames = ['app1.yml','app2.yml','app3.yml']
grasp_filenames = ['grasp1.yml','grasp2.yml','grasp3.yml']
transport_filenames = ['transport1.yml','transport2.yml','transport3.yml']

skill_filenames = {}
skill_filenames['approach'] = approach_filenames
skill_filenames['grasp'] = grasp_filenames
skill_filenames['transport'] = transport_filenames

skill_objs = {'approach':['link'], 'grasp':['link'], 'transport':['link','node']}
skill_fixed = {'approach':[], 'grasp':['link'], 'transport':['link']}

# load data for each skill
for name,filenames in skill_filenames.items():

    data,params,num_weights,goals = grid.LoadDataDMP(filenames)

    # create the skill object
    skill = grid.RobotSkill(name=name,action_k=ak,goal_k=gk,data=data,params=params,goals=goals)#,num_weights=num_weights)

    skill.save(name+"_skill.yml")

print "... Done creating skills."

try:
    while not rospy.is_shutdown():
        pass
except rospy.ROSInterruptException,e:
    pass