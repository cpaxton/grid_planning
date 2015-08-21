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
    def __init__(self,gmm,features,cmd_topic):
        self.gmm = gmm
        self.features = features
        self.active_skill = None
        
        self.skill_sub = rospy.Subscriber(SKILL_TOPIC,std_msgs.msg.String,self.skill_cb)

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
