#!/usr/bin/env

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

# load data for each skill
for name,filenames in skill_filenames:

    for filename in filenames:


    # create the skill object
    skill = RobotSkill(name=name,action_k=ak,goal_k=gk)

    skill.save(name+"_skill.yml")
