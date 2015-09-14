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
take_filenames = ['ur5_data/take1.yml','ur5_data/take1b.yml','ur5_data/take2.yml','ur5_data/take3.yml']
close_filenames = ['data/close1.yml']
lift_filenames = ['data/lift1.yml','data/lift2.yml']

skill_filenames = {}
skill_filenames['approach'] = approach_filenames
skill_filenames['grasp'] = grasp_filenames
skill_filenames['transport'] = transport_filenames
skill_filenames['disengage'] = disengage_filenames
skill_filenames['release'] = release_filenames
skill_filenames['align'] = align_filenames
skill_filenames['place'] = place_filenames
#skill_filenames['init'] = init_filenames

#skill_objs = {'approach':['time','link'], 'grasp':['time','link'], 'transport':['time','link','node'], 'disengage':['time','link']}
skill_objs = {'approach':['time','link'], 'grasp':['time','link'], 'transport':['time','node'], 'align':['time','node'], 'release':['time','node'],'place':['time','node'],'disengage':['time','link']}#, 'init':[]}
skill_fixed = {'approach':[], 'grasp':[], 'transport':['link'], 'align':['link'],'place':['link'],'disengage':[],'release':['link']} #,'init':[]}

all_params = []

# load data for each skill
for name,filenames in skill_filenames.items():

    # create some GMMs for gripper stuff too!
    # this is just a convenience thing; they really SHOULD be the same as the ones we're using above
    data,params,num_weights,goals = grid.LoadDataDMP(filenames,skill_objs[name] + ['gripper'],manip_objs=skill_fixed[name])

    training_data = np.array(data[0][1])
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i][1]))

    if not name=='grasp':
        all_params += params

    # create the skill object
    skill = grid.RobotSkill(name=name,action_k=ak,goal_k=gk,data=training_data,objs=(skill_objs[name]+['gripper']),manip_objs=skill_fixed[name],params=params,goals=goals)#,num_weights=num_weights)

    skill.save(name+"_skill.yml")

print "... Done creating skills."

#skill = grid.RobotSkill(name='default',action_k=ak,goal_k=gk,data=training_data,params=params,goals=goals)
#skill.save('default.yml')

#print '... Done creating default.'

try:
    while not rospy.is_shutdown():
        pass
except rospy.ROSInterruptException,e:
    pass
