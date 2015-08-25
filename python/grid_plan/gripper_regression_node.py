import rospy
import grid
import oro_barrett_msgs
from oro_barrett_msgs.msg import BHandCmd
import std_msgs
import copy

SKILL_TOPIC = "current_skill"

class GripperRegressor:

    '''
    reads in a std_msgs/string object
    '''
    def skill_cb(self,msg):
        self.active_skill = msg.data

    '''
    sets up the node based on a GMM and the gripper topic
    '''
    def __init__(self,cmd_topic,skill_topic,features):
        #self.gmm = gmm
        #self.features = features
        self.gmms = {}
        self.skills = {}
        self.active_skill = None
        
        self.skill_sub = rospy.Subscriber(skill_topic,std_msgs.msg.String,self.skill_cb)
        self.cmd_pub = rospy.Publisher(cmd_topic,oro_barrett_msgs.msg.BHandCmd)
        self.features = grid.RobotFeatures(
                base_link=features.base_link,
                end_link=features.end_link,
                world_frame=features.world_frame,
                gripper_topic=features.gripper_topic,
                objects=features.objects,
                indices=features.indices,
                robot_description_param=features.robot_description_param,
                dof=features.dof
                )

    '''
    tick()
    update robot and world state appropriately
    compute features
    publish a bhandcmd message
    '''
    def tick(self):
        if self.active_skill in self.gmms:
            pass
            # create new data with gripper indices missing

            # use pypr to fill in missing data
    
    '''
    start()
    loop this node in a separate thread
    publish gripper command messages as necessary
    '''
    def start(self):
        pass

    def addSkill(self,skill):
        #skill.index

        print skill.name
        print skill.objs
        for obj in skill.objs:
            self.features.AddObject(obj)
        print self.features.indices

        self.skills[skill.name] = skill
        self.gmms[skill.name] = skill.gripper_model

