#!/usr/bin/env python

import rospy
import tf
from subprocess import Popen

procs = []

test_cmd = ["rosrun","grid_plan","task_test"]
reset_cmd = ["rosrun","grid_experiments","reset.py"]

args = ["_step_size:=0.5",
        "_iter:=15",
        "_ntrajs:=200",
        "_starting_horizon:=5",
        "_max_horizon:=5",
        "_detect_collisions:=true",
        "_wait:=0",
        "_collisions_verbose:=0",
        "_base_model_norm:=0.0001",
        "_model_norm_step:=1",
        "_update_horizon:=0.0001"
        ]

try:
    for i in range(1,11):

        name = 'double%d:=true'%(i)
        print name

        ''' --------------------- '''
        '''     RESET THE ARM     '''
        ''' --------------------- '''

        print "[%d] Resetting to standard start configuration..."%(i)
        reset_proc = Popen(reset_cmd)
        reset_proc.wait()

        rospy.sleep(0.5)

        ''' --------------------- '''
        ''' LAUNCH THE EXPERIMENT '''
        ''' --------------------- '''

        launch_cmd = ['roslaunch','grid_experiments','ascent_experiments.launch',name]

        proc = Popen(launch_cmd)
        procs.append(proc)

        rospy.sleep(2.0)

        ''' --------------------- '''
        '''     START PLANNING    '''
        ''' --------------------- '''

        print "[%d] Starting test..."%(i)
        test_proc = Popen(test_cmd+args)

        test_proc.wait()

        print "[%d] Done test!"%(i)



        break

except Exception, e:
    print e
finally:
    for proc in procs:
        proc.poll()
        if proc.returncode is None:
            print " - Killing %s"%(str(proc))
            proc.terminate()

