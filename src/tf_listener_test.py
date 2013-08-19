#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_calibration')
import rospy
import math
import tf
import numpy
from tf_conversions import posemath
from geometry_msgs.msg import Pose
from clopema_utilities import np_bridge

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
	

if __name__ == '__main__':
	rospy.init_node('tf_listener_test')
	listener = tf.TransformListener()
	while not rospy.is_shutdown():
		now = rospy.Time(0)
		listener.waitForTransform('/base_link','/r2_gripper',now,rospy.Duration(1.5))
		(trans,rot) = listener.lookupTransform('/base_link','/r2_gripper',now)
		print 'Trans = ' + str(trans) + ' rot = ' + str(rot)
		print quat2rot(rot)
		rospy.sleep(1)
