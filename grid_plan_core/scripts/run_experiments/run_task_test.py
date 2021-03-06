#!/usr/bin/env python

import rospy
import tf
from subprocess import Popen
import tf_conversions.posemath as pm

def lookup(frame1,frame2):
    global listener
    done = False
    (trans,rot) = (None,None)
    while not done:
        try:
            (trans,rot) = listener.lookupTransform(frame1,frame2,rospy.Time(0))
            done = True
        except Exception, e:
            print e
            rospy.sleep(0.1)

    return (trans,rot)
        
rospy.init_node('run_task_test_master')
listener = tf.TransformListener()
procs = []
final = [[],[],[],[],[],[],[],[]]

#executables = ["simple_test","simple_test","task_test","task_test",
#        "simple_test","simple_test","task_test","task_test"]
executables = ["task_test","task_test","task_test","task_test",
        "task_test","task_test","task_test","task_test"]

reset_cmd = ["rosrun","grid_experiments","reset.py"]
target_cmd = ["roslaunch","grid_plan","test_targets.launch"]

task_args = ["_step_size:=0.5",
        "_iter:=15",
        "_ntrajs:=300",
        "_starting_horizon:=5",
        "_max_horizon:=5",
        "_detect_collisions:=true",
        "_wait:=0",
        "_collisions_verbose:=0",
        "_base_model_norm:=0.001",
        "_model_norm_step:=1",
        "_update_horizon:=0.01",
        "_compute_statistics:=true",
        "_collision_detection_step:=2",
        "_distribution_noise:=1e-8",
        "_replan_depth:=0",
        "_execute_depth:=5",
        "_fixed_distribution_noise:=true",
        "_random_transitions:=false",
        "_randomize:=true"
        ]
single_args = ["_step_size:=0.5",
        "_iter:=15",
        "_ntrajs:=300",
        "_starting_horizon:=2",
        "_max_horizon:=1",
        "_detect_collisions:=true",
        "_wait:=0",
        "_collisions_verbose:=0",
        "_base_model_norm:=0.001",
        "_model_norm_step:=1",
        "_update_horizon:=0.01",
        "_compute_statistics:=true",
        "_collision_detection_step:=2",
        "_distribution_noise:=1e-8",
        "_replan_depth:=1",
        "_execute_depth:=5",
        "_fixed_distribution_noise:=true",
        "_random_transitions:=true",
        "_randomize:=true"
        ]
single_lookahead_args = ["_step_size:=0.5",
        "_iter:=15",
        "_ntrajs:=300",
        "_starting_horizon:=5",
        "_max_horizon:=5",
        "_detect_collisions:=true",
        "_wait:=0",
        "_collisions_verbose:=0",
        "_base_model_norm:=0.001",
        "_model_norm_step:=1",
        "_update_horizon:=0.01",
        "_compute_statistics:=true",
        "_collision_detection_step:=2",
        "_distribution_noise:=1e-8",
        "_replan_depth:=0",
        "_execute_depth:=5",
        "_fixed_distribution_noise:=true",
        "_random_transitions:=true",
        "_randomize:=true"
        ]
no_lookahead_args = ["_step_size:=0.5",
        "_iter:=15",
        "_ntrajs:=300",
        "_starting_horizon:=2",
        "_max_horizon:=1",
        "_detect_collisions:=true",
        "_wait:=0",
        "_collisions_verbose:=0",
        "_base_model_norm:=0.001",
        "_model_norm_step:=1",
        "_update_horizon:=0.01",
        "_compute_statistics:=true",
        "_collision_detection_step:=2",
        "_distribution_noise:=1e-8",
        "_replan_depth:=1",
        "_execute_depth:=5",
        "_fixed_distribution_noise:=true",
        "_random_transitions:=false",
        "_randomize:=true"
        ]

human_arg = ["_test:=0"];
auto_arg = ["_test:=1"];

args = [single_args+human_arg,single_lookahead_args+human_arg,no_lookahead_args+human_arg,task_args+human_arg,
        single_args+auto_arg,single_lookahead_args+auto_arg,no_lookahead_args+auto_arg,task_args+auto_arg]

try:
    for test in [3,7,1,2,0,5,6,4]: #range(0,8):
    #for test in [1,2,0,5,6,4]: #range(0,8):
    #for test in [0,5,6,4]: #range(0,8):
    #for test in [6,4]: #range(0,8):

        print "Running test case %d..."%test

        test_cmd = ["rosrun","grid_plan"] + [executables[test]]
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
            target_proc = Popen(target_cmd)
            procs.append(target_proc)

            rospy.sleep(2.0)

            ''' --------------------- '''
            '''     START PLANNING    '''
            ''' --------------------- '''

            print "[%d] Starting test..."%(i)
            test_proc = Popen(test_cmd+args[test])

            test_proc.wait()

            print "[%d] Done test!"%(i)

            #(trans1,rot1) = lookup("gbeam_link_1/gbeam_link","gbeam_node_1/gbeam_node")
            #(trans2,rot2) = lookup("gbeam_link_1/gbeam_link","gbeam_node_2/gbeam_node")
            (trans1,rot1) = lookup("gbeam_link_1/gbeam_link","target1")
            (trans2,rot2) = lookup("gbeam_link_1/gbeam_link","target2")

            t1 = pm.fromTf((trans1,rot1))
            t2 = pm.fromTf((trans2,rot2))

            proc.terminate()
            target_proc.terminate()
            rospy.sleep(1.0)

            if t1.p.Norm() < t2.p.Norm():
                final[test].append(t1)
            else:
                final[test].append(t2)

except Exception, e:
    print e
finally:
    for proc in procs:
        proc.poll()
        if proc.returncode is None:
            print " - Killing %s"%(str(proc))
            proc.terminate()

    print final
    i = 0
    for final_test in final:
        i += 1
        print "test case %d:"%(i)
        for pose in final_test:
            print "%f %f %f %f"%(pose.p.Norm(), pose.p.x(), pose.p.y(), pose.p.z())


