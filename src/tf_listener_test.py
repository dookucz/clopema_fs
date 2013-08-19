#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_calibration')
import rospy
import math
import tf
from tf_conversions import posemath
from geometry_msgs.msg import Pose
from clopema_utilities import np_bridge

if __name__ == '__main__':
	rospy.init_node('tf_listener_test')
	listener = tf.TransformListener()
	while not rospy.is_shutdown():
		now = rospy.Time(0)
		listener.waitForTransform('/base_link','/r2_gripper',now,rospy.Duration(1.5))
		(trans,rot) = listener.lookupTransform('/base_link','/r2_gripper',now)
		print 'Trans = ' + str(trans) + ' rot = ' + str(rot)
		rospy.sleep(1)
