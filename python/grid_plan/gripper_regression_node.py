import grid
from oro_barrett_msgs.msg import BHandCmd


class GripperRegressor:

    '''
    sets up the node based on a GMM and the gripper topic
    '''
    def __init__(self,gmm,features,cmd_topic):
        self.gmm = gmm
        self.features = features

    '''
    tick()
    update robot and world state appropriately
    compute features
    publish a bhandcmd message
    '''
    def tick(self):
        pass
