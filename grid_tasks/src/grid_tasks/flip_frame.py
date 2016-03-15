#!/usr/bin/env python

import rospy
import tf
import tf_conversions as tfc
import PyKDL as pk

def flip_rotation_frame(trans, rot):
    f = tfc.fromTf((trans, rot))
    ir = f.M.Inverse()

    return tfc.toTf(tfc.Frame(ir, f.p))


if __name__ == '__main__':

    print flip_rotation_frame((0, 0, 0), (0, 1, 0, 1));
