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
import tf
from tf_conversions import posemath
from geometry_msgs.msg import Pose
from clopema_utilities import np_bridge

goals = []

def quat2rot(quat):
    w = quat[3]
    x = quat[0]
    y = quat[1]
    z = quat[2]
    Nq = w*w + x*x + y*y + z*z
    if Nq > 0.0:
        s = 2/Nq
    else:
        s = 0.0
    X = x*s; Y = y*s; Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    return numpy.matrix([[1.0-(yY+zZ), xY-wZ, xZ+wY],[xY+wZ,1.0-(xX+zZ),yZ-wX],[xZ-wY,yZ+wX,1.0-(xX+yY)]])

def pick_joints(ud):
    ud.goal = goals[ud.n]
    print goals[ud.n]
    return 'succeeded'

def write_force(msg):
    MAX_COUNT = 10000
    global d
    global count
    global pos_num
    if count >= 0:
        if count < MAX_COUNT:
            d.write(str(pose_num) + ';')
            d.write("%15.5f" % msg.header.stamp.to_sec() + ";")
            d.write(str(msg.wrench.force.x) + ";")
            d.write(str(msg.wrench.force.y) + ";")
            d.write(str(msg.wrench.force.z) + ";")
            d.write(str(msg.wrench.torque.x) + ";")
            d.write(str(msg.wrench.torque.y) + ";")
            d.write(str(msg.wrench.torque.z) + ";\n")
            count += 1
        else:
            count = -1
            pose_num += 1

def store_data(msg, ud):
    global pose_num
    global count
    count = 0
    print ud.goal
    listener = tf.TransformListener()
    with open("data_rot", "a") as f:
        now = rospy.Time(0)
        listener.waitForTransform('/base_link','/r2_force_sensor',now,rospy.Duration(1.5))
        (trans,rot) = listener.lookupTransform('/base_link','/r2_force_sensor',now)
        f.write(str(pose_num) + ':\n' + str(quat2rot(rot)) + '\n\n')    
    f.close()
    return

def to_rad(deg):
    return deg * math.pi / 180

def main():
    global d
    global pose_num
    global count

    count = -1
    pose_num = 1

    rospy.init_node('sm_test')
    d = open("data_fs", "w")
    # for t in numpy.arange(-to_rad(195), -to_rad(100) + 0.001, math.pi / 3):            # max range -to_rad(195), to_rad(195) + 0.001
    #     for b in numpy.arange(to_rad(-40), to_rad(80) + 0.001, math.pi / 3):	      # max range  to_rad(-40), to_rad(175) + 0.001
    #         for r in numpy.arange(-to_rad(145), to_rad(0) + 0.001, math.pi / 3):    # max range -to_rad(145), to_rad(145) + 0.001
    #             goal = MA1400JointState()
    #             goal.t = t
    #             goal.b = b
    #             goal.r = r
    #             goals.append(copy.deepcopy(goal))
    ###############
    # 1
    offset = -10
    goal = MA1400JointState()
    goal.r = to_rad(-45+offset)
    goal.b = to_rad(48)
    goal.t = 0
    goals.append(copy.deepcopy(goal))
    # 2
    goal = MA1400JointState()
    goal.r = to_rad(offset)
    goal.b = to_rad(48)
    goal.t = 0
    goals.append(copy.deepcopy(goal))
    # 3
    goal = MA1400JointState()
    goal.r = to_rad(45+offset)
    goal.b = to_rad(48)
    goal.t = 0
    goals.append(copy.deepcopy(goal))
    ##############
    
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
        smach.Sequence.add('PAUSE1', PauseState(1))
        smach.Sequence.add('TOPIC_STATE', TopicState('/netft_data', WrenchStamped, store_data, input_keys=['goal']))
        smach.Sequence.add('PAUSE2', PauseState(12),transitions={'succeeded':'counter'})
        smach.Sequence.add('POWER_OFF', SetServoPowerOffState())
       
    sis = smach_ros.IntrospectionServer('sm_test', sm, '/SM_ROOT')
    sis.start()
    os.system('clear')
    outcome = sm.execute()
    sis.stop()
    d.close()

if __name__ == '__main__':
    main()
