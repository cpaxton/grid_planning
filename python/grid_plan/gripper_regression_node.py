import rospy
import grid
from oro_barrett_msgs.msg import BHandCmd
import std_msgs

SKILL_TOPIC = "current_skill"

class GripperRegressor:

    '''
    reads in a std_msgs/string object
    '''
    def skill_cb(msg):
        self.active_skill = msg.data

    '''
    sets up the node based on a GMM and the gripper topic
    '''
    def __init__(self,cmd_topic,skill_topic):
        #self.gmm = gmm
        #self.features = features
        self.gmms = {}
        self.active_skill = None
        
        self.skill_sub = rospy.Subscriber(skill_topic,std_msgs.msg.String,self.skill_cb)
        self.cmd_pub = rospy.Publisher(cmd_topic,oro_barrett_msgs.msg.BHandCmd)

    '''
    tick()
    update robot and world state appropriately
    compute features
    publish a bhandcmd message
    '''
    def tick(self):
        pass
    
    '''
    start()
    loop this node in a separate thread
    publish gripper command messages as necessary
    '''
    def start(self):
        pass

    def addSkill(self,skill):
        self.gmms[skill.name] = skill.gripper_model
