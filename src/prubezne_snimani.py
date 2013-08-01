#!/usr/bin/env python
import roslib; roslib.load_manifest('clopema_calibration')
import rospy, smach, smach_ros, math, copy, tf, PyKDL, os, shutil, numpy
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from tf_conversions import posemath
from clopema_smach import *
from geometry_msgs.msg import *
from smach import State
from clopema_planning_actions.msg import MA1400JointState

wrench = WrenchStamped()

goals = []

def pick_joints(ud):
    ud.goal = goals[ud.n]
    print goals[ud.n]
    return 'succeeded'

def write_force(msg):
    global d
    d.write("%15.5f" % msg.header.stamp.to_sec() + ";")
    d.write(str(msg.wrench.force.x) + ";")
    d.write(str(msg.wrench.force.y) + ";")
    d.write(str(msg.wrench.force.z) + ";")
    d.write(str(msg.wrench.torque.x) + ";")
    d.write(str(msg.wrench.torque.y) + ";")
    d.write(str(msg.wrench.torque.z) + ";\n")

def store_data(msg, ud):
    wrench = msg
    print msg.wrench.force
    print ud.goal
    with open("times", "a") as f:
        f.write("%15.5f" % msg.header.stamp.to_sec() + "; ")
    f.close()
    return

def to_rad(deg):
    return deg * math.pi / 180

def main():
    global d
    rospy.init_node('sm_test')
    d = open("data", "a")
    for t in numpy.arange(-to_rad(195), -to_rad(100) + 0.001, math.pi / 3):            # max range -to_rad(195), to_rad(195) + 0.001
        for b in numpy.arange(to_rad(-40), to_rad(80) + 0.001, math.pi / 3):	      # max range  to_rad(-40), to_rad(175) + 0.001
            for r in numpy.arange(-to_rad(145), to_rad(0) + 0.001, math.pi / 3):    # max range -to_rad(145), to_rad(145) + 0.001
                goal = MA1400JointState()
                goal.t = t
                goal.b = b
                goal.r = r
                goals.append(copy.deepcopy(goal))
    
    print len(goals)
    
    rospy.Subscriber('/netft_data',WrenchStamped,write_force)
    
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
        smach.Sequence.add('PAUSE', PauseState(5))
        smach.Sequence.add('TOPIC_STATE', TopicState('/netft_data', WrenchStamped, store_data, input_keys=['goal']),
                           transitions={'succeeded':'counter'})
        smach.Sequence.add('POWER_OFF', SetServoPowerOffState())
       
    sis = smach_ros.IntrospectionServer('sm_test', sm, '/SM_ROOT')
    sis.start()
    os.system('clear')
    outcome = sm.execute()
    sis.stop()
    f.close()

if __name__ == '__main__':
    main()
