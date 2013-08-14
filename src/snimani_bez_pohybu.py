#!/usr/bin/env python
#
#   Spustit jako:
#       rosrun clopema_fs prubezne_snimani.py | tee output
#       view_data.py dostane ze souboru "output" potrebna data
#
import roslib; roslib.load_manifest('clopema_calibration')
import rospy, smach, smach_ros, math, copy, tf, PyKDL, os, shutil, numpy
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from tf_conversions import posemath
from geometry_msgs.msg import *
from clopema_planning_actions.msg import MA1400JointState
import time
import sys

wrench = WrenchStamped()

def write_force(msg):
    global d
    global start_time

    d.write("%15.5f" % msg.header.stamp.to_sec() + ";")
    d.write(str(msg.wrench.force.x) + ";")
    d.write(str(msg.wrench.force.y) + ";")
    d.write(str(msg.wrench.force.z) + ";")
    d.write(str(msg.wrench.torque.x) + ";")
    d.write(str(msg.wrench.torque.y) + ";")
    d.write(str(msg.wrench.torque.z) + ";\n")
    if msg.header.stamp.to_sec() > (start_time+30): # cas cekani 30 s
        d.close()
        sys.exit(0)

def main():
    global d
    global start_time
    d = open("data_no_movement","a")
    rospy.init_node('sm_test')
    start_time = time.mktime(time.localtime())
    rospy.Subscriber('/netft_data',WrenchStamped,write_force)
    rospy.spin()

    f.close()

if __name__ == '__main__':
    main()
