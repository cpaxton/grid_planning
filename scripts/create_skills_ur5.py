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
take_filenames = ['ur5_data/take1.yml','ur5_data/take2.yml','ur5_data/take3.yml']
align_filenames = ['ur5_data/align1.yml','ur5_data/align2.yml','ur5_data/align3.yml']
grab_filenames = ['ur5_data/grab1.yml','ur5_data/grab2.yml','ur5_data/grab3.yml']
gc_filenames = ['ur5_data/gc1.yml','ur5_data/gc2.yml','ur5_data/gc3.yml']
close_filenames = ['ur5_data/close1.yml','ur5_data/close2.yml']
lift_filenames = ['ur5_data/lift1.yml','ur5_data/lift2.yml','ur5_data/lift3.yml']

skill_filenames = {}
skill_filenames['align2'] = align_filenames
skill_filenames['take'] = take_filenames
skill_filenames['grab'] = grab_filenames
skill_filenames['gc'] = gc_filenames
skill_filenames['close'] = close_filenames
skill_filenames['lift'] = lift_filenames

skill_objs = {'take':['time','tool'], 'close':['time','tool'], 'lift':['time','vise'],'grab':['time','vise'],'gc':['time','vise'],'align2':['time','tool']}
skill_fixed = {'take':[], 'close':[], 'lift':[],'grab':[],'gc':[],'align2':[]}

# load data for each skill
for name,filenames in skill_filenames.items():

    # create some GMMs for gripper stuff too!
    # this is just a convenience thing; they really SHOULD be the same as the ones we're using above
    data,params,num_weights,goals = grid.LoadDataDMP(filenames,skill_objs[name],manip_objs=skill_fixed[name],preset='ur5')

    training_data = np.array(data[0][1])
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i][1]))

    # create the skill object
    if name=='close':
        skill = grid.RobotSkill(name=name,action_k=ak,goal_k=gk,data=training_data,objs=(skill_objs[name]),manip_objs=skill_fixed[name],params=params,goals=goals,normalize=True)
    else:
        skill = grid.RobotSkill(name=name,action_k=ak,goal_k=gk,data=training_data,objs=(skill_objs[name]),manip_objs=skill_fixed[name],params=params,goals=goals,normalize=True)

    skill.save(name+"_skill.yml")

print "... Done creating skills."

try:
    while not rospy.is_shutdown():
        pass
except rospy.ROSInterruptException,e:
    pass
