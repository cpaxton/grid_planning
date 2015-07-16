
'''
Demonstration
struct to hold robot task demonstration data.
'''
class Demonstration:
    def __init__(self):
        self.joint_p = []
        self.joint_v = []
        self.joint_t = []
        self.gripper_cmd = []
        self.tform = {}
        self.world_t = []


