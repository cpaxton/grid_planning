from dtw import dtw
import numpy as np
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

'''
Demonstration
struct to hold robot task demonstration data.

Provides:
    - get_features: computes alignment between different data sources and returns a simplified set of data
'''
class Demonstration:
    def __init__(self,filename=None):
        self.joint_p = []
        self.joint_v = []
        self.joint_t = []
        self.gripper_cmd = []
        self.gripper_t = []
        self.tform = {}
        self.world_t = []

        if not filename==None:
            stream = file(filename,'r')
            demo = yaml.load(stream,Loader=Loader)
            self.joint_p = demo.joint_p
            self.joint_v = demo.joint_v
            self.joint_t = demo.joint_t
            self.gripper_cmd = demo.gripper_cmd
            self.tform = demo.tform
            self.world_t = demo.world_t

    '''
    get_features returns a task-space view of the demonstration
    and a merged version of the trajectory
    '''
    def get_features(self,frames=[]):

        # make times into floats
        jt = [t.to_sec() for t in self.joint_t]
        gt = [t.to_sec() for t in self.gripper_t]
        wt = [t.to_sec() for t in self.world_t]

        jidx = []
        widx = []

        # select subsamples of times/other variables to create training data
        jd,jc,jpath = dtw(gt,jt)
        wd,wc,wpath = dtw(gt,wt)
        for i in range(len(gt)):
            jidx.append(np.argmin(jc[i,:]))
            widx.append(np.argmin(wc[i,:]))
        
        jp = [self.joint_p[i] for i in jidx]
        jv = [self.joint_v[i] for i in jidx]
        jts = [self.joint_t[i] for i in jidx]
        wts = [self.world_t[i] for i in widx]
        
        # go through all these variables and choose the right ones
        fx = []
        x = []
        u = []
        for i in range(len(gt)):
            features = [] + self.gripper_cmd[i]
            for frame1,frame2 in frames:
                f1 = self.tform[frame1][widx[i]]
                f2 = self.tform[frame2][widx[i]]
                transform = f1.Inverse() * f2

                features += transform.M.GetRPY()
                features += transform.p
                features += [transform.p.Norm()]
            fx.append(features)
            x.append(jp[i] + self.gripper_cmd[i])
            u.append(jv[i] + self.gripper_cmd[i])

        return fx, x, u, gt

'''
LoadYaml
Really simple function to quickly load from a yaml file
'''
def LoadYaml(filename):
    stream = file(filename,'r')
    demo = yaml.load(stream,Loader=Loader)

    return demo
