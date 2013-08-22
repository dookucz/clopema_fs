#!/usr/bin/env python
#
#   Spustit jako:
#       rosrun clopema_fs prubezne_snimani.py | tee output
#       view_data.py dostane ze souboru "output" potrebna data
#
import roslib; roslib.load_manifest('clopema_fs')
import rospy, smach, smach_ros, math, copy, tf, PyKDL, os, shutil, numpy
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from tf_conversions import posemath
from clopema_smach import *
from geometry_msgs.msg import *
from smach import State
from clopema_planning_actions.msg import MA1400JointState

goals = []

def pick_joints(ud):
    ud.goal = goals[ud.n]
    print goals[ud.n]
    return 'succeeded'

def to_rad(deg):
    return deg * math.pi / 180

def main():
    global d
    rospy.init_node('sm_test')
    t = 0
    b = 0
    for r in numpy.arange(-to_rad(50), to_rad(50) + 0.001, math.pi / 6):            # max range -to_rad(195), to_rad(195) + 0.001
		goal = MA1400JointState()
		goal.t = t
		goal.b = b
		goal.r = -r
		goals.append(copy.deepcopy(goal))
    
    print len(goals)
    
    sm = smach.Sequence(outcomes=['succeeded', 'preempted', 'aborted'], connector_outcome='succeeded')
    sm_plan_joint = gensm_plan_vis_exec(Plan1ToJointsState(), input_keys=['goal'], confirm=False)
    sm_plan_joint.userdata.robot = 2
    
    sm.userdata.n = -1
    with sm:
        smach.Sequence.add('counter', CounterState(len(goals)),
                           transitions={'aborted':'POWER_OFF'})
        smach.Sequence.add('pick_joints', FunctionApplicationState(pick_joints, input_keys=['n'],
                                        output_keys=['goal'], outcomes=['succeeded']))
        smach.Sequence.add('GOTO', sm_plan_joint,
                           transitions={'aborted':'counter'})
        smach.Sequence.add('PAUSE', PauseState(5), transitions={'succeeded':'counter'})
        smach.Sequence.add('POWER_OFF', SetServoPowerOffState())
       
    sis = smach_ros.IntrospectionServer('sm_test', sm, '/SM_ROOT')
    sis.start()
    os.system('clear')
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    main()
